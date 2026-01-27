from fastapi import FastAPI, HTTPException, UploadFile, File, Form
from pydantic import BaseModel
from typing import List, Dict, Any, Optional
import json
import os
import yaml
import time
import shutil
import pdb
from pathlib import Path
from src.core.graph_loader import load_stair_graph_and_elevator_graphs
from src.schedulers.batch_scheduler import init_elevators, get_robot_status_real_time, merge_paths, Robot, \
    BatchScheduler, Task
from src.core.template_to_topo import building_graph

# BASE_PATH = os.path.join(os.path.dirname(os.path.dirname(__file__)), "src", "core", "data")
BASE_PATH = Path(__file__).parent.parent / "src" / "core" / "data"

app = FastAPI()

# 全局变量
batch_scheduler: Optional[BatchScheduler] = None
task_counter = 0


# 原写死版本
# # 初始化全局调度器（避免每次请求都重新初始化）
# stair_graph, add_1E1_graph, add_1E2_graph, add_2E1_graph, add_2E2_graph, add_3E1_graph, add_3E2_graph, _ = initial_six_graphs(
#     speed_land=1.5, speed_stair=0.5
# )
# elevators = init_six_elevators()
# robots = [
#     Robot(0, "dog", "1_1_Left_1"),
#     Robot(1, "dog", "1_1_Left_1"),
#     Robot(2, "human", "1_1_Left_1"),
#     Robot(3, "human", "1_1_Left_1"),
# ]
# elevator_graphs = {
#     "1_E1": add_1E1_graph, "1_E2": add_1E2_graph,
#     "2_E1": add_2E1_graph, "2_E2": add_2E2_graph,
#     "3_E1": add_3E1_graph, "3_E2": add_3E2_graph
# }
# batch_scheduler = BatchScheduler(robots, elevators, stair_graph, elevator_graphs)
# task_counter = 0


# 配置模型
class MapConfig(BaseModel):
    speed_land: float = 1.5
    speed_stair: float = 0.5
    elevator_configs: Dict[str, Any]  # 电梯配置
    robot_configs: List[Dict[str, Any]]  # 机器人配置
    graph_files: Dict[str, str]  # 各种图形的配置文件路径


# 定义输入任务的Pydantic模型（单个任务）
class SingleTaskInput(BaseModel):
    task_id: int  # 任务ID
    skill: str  # 任务类型（如 "dog" 或 "human"）
    priority: int  # 任务优先级（如 "1", "2"）
    start_position: str  # 起点（如 "4_3_A"）
    target_position: str  # 终点（如 "6_3_G"）


# 定义批量输入模型（多个任务）
class BatchTaskInput(BaseModel):
    tasks: List[SingleTaskInput]  # 任务列表


class AssignmentOutput(BaseModel):
    task_id: int  # 任务ID
    robot_id: int  # 分配的机器人ID
    path: List[str]  # 预计路径（起点到终点的路径点列表）
    estimated_time: float  # 预估完成时间（秒）


@app.get("/")
async def root():
    return {"message": "Robot Path API is alive!", "docs": "/docs"}


@app.post("/initialize-map")
async def initialize_map(
        campus_name: str = Form(...),  # 从表单数据获取校园名称
        yaml_files: list[UploadFile] = File(...)  # 从文件上传获取多个YAML文件
):
    """
    初始化地图和调度器
    输入:
        - campus_name: 校园名称（用于创建存储目录）
        - yaml_files: 上传的YAML文件列表（可能包含多个文件）
    输出: 初始化状态
    """
    global batch_scheduler

    try:
        # 1. 创建校园数据目录（如果不存在）
        campus_dir = BASE_PATH / campus_name
        campus_dir.mkdir(parents=True, exist_ok=True)

        # 2. 保存上传的YAML文件到校园目录
        saved_file_paths = []
        for yaml_file in yaml_files:
            file_path = campus_dir / "yaml" / yaml_file.filename
            with open(file_path, "wb") as buffer:
                shutil.copyfileobj(yaml_file.file, buffer)
            saved_file_paths.append(file_path)

        building_graph(campus_name)
        print("Building!")

        MERGED_YAML = os.path.join(BASE_PATH, campus_name, "merged_nodes.yaml")
        print(MERGED_YAML)
        # 初始化图与对象
        stair_graph, elevator_graphs = load_stair_graph_and_elevator_graphs(
            merged_nodes_yaml=MERGED_YAML,
            elevator_xy_tol=0.25,  # (x,y) 判同一电梯的容差，可调；严格相同可设 0.0
            ensure_legacy_keys=True,  # 给 batch_scheduler 补齐 1_E1..3_E2，避免 KeyError
            speed_land=1.5,
            speed_stair=1.0
        )

        print(elevator_graphs)
        elevators_name = []
        for key in elevator_graphs.keys():
            elevators_name.append(key)

        # 你现在 scheduler 里仍然用 init_six_elevators（固定6台），先保留也能跑。
        # 如果你后续要严格“只用真实存在的电梯”，我们再把 scheduler 里硬编码那块改成动态遍历 elevator_graphs。
        elevators = init_elevators(elevators_name)
        # pdb.set_trace()

        robots = [
            Robot(0, "dog", "4_1_p1", campus_name=campus_name),
            Robot(1, "dog", "4_1_p1", campus_name=campus_name),
            Robot(2, "human", "4_1_p1", campus_name=campus_name),
            Robot(3, "human", "4_1_p1", campus_name=campus_name),
        ]
        # elevator_graphs = {
        #     "1_E1": add_1E1_graph, "1_E2": add_1E2_graph,
        #     "2_E1": add_2E1_graph, "2_E2": add_2E2_graph,
        #     "3_E1": add_3E1_graph, "3_E2": add_3E2_graph
        # }

        # 创建批量调度器
        batch_scheduler = BatchScheduler(robots, elevators, stair_graph, elevator_graphs, campus_name)

        return {"status": "success", "message": "Map and scheduler initialized successfully"}

    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))


@app.post("/schedule-tasks", response_model=List[AssignmentOutput])
async def schedule_tasks(batch_input: BatchTaskInput):
    """
    批量调度任务接口
    输入：多个任务（任务类型、优先级、起点、终点）
    输出：每个任务的分配结果（机器人ID、路径、预估时间）
    """
    global batch_scheduler

    print(batch_input)

    try:
        # 将API输入转换为Task对象列表
        batch_tasks = []
        for task_input in batch_input.tasks:
            batch_tasks.append(
                Task(
                    tid=task_input.task_id,
                    skill=task_input.skill,
                    start=task_input.start_position,
                    target=task_input.target_position,
                    priority=task_input.priority
                )
            )

        # 调用调度器分配任务
        assignments = batch_scheduler.schedule_batch(batch_tasks)

        # 格式化输出
        output = []
        for assignment in assignments:
            path1 = assignment["pick_path_info"]["path"]
            path2 = assignment["deliver_path_info"]["path"]

            full_path = merge_paths(path1, path2)
            estimated_time = assignment["end_time"] - assignment["start_time"]

            output.append(
                AssignmentOutput(
                    task_id=assignment["task"].id,
                    robot_id=assignment["robot_id"],
                    path=full_path,
                    estimated_time=estimated_time
                )
            )

        return output

    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))


@app.get("/robot-status")
async def get_robot_status():
    """
    查询机器人状态接口
    """
    status_data = get_robot_status_real_time(batch_scheduler)  # 假设你有这个函数
    return {"status": "success", "data": status_data}
