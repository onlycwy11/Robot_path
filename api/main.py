from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from typing import List, Dict, Any
import json
from src.schedulers.batch_scheduler import initial_six_graphs, init_six_elevators, get_robot_status_real_time, merge_paths, Robot, BatchScheduler, Task
import time

app = FastAPI()

# 初始化全局调度器（避免每次请求都重新初始化）
stair_graph, add_1E1_graph, add_1E2_graph, add_2E1_graph, add_2E2_graph, add_3E1_graph, add_3E2_graph, _ = initial_six_graphs(
    speed_land=1.5, speed_stair=0.5
)
elevators = init_six_elevators()
robots = [
    Robot(0, "dog", "1_1_Left_1"),
    Robot(1, "dog", "1_1_Left_1"),
    Robot(2, "human", "1_1_Left_1"),
    Robot(3, "human", "1_1_Left_1"),
]
elevator_graphs = {
    "1_E1": add_1E1_graph, "1_E2": add_1E2_graph,
    "2_E1": add_2E1_graph, "2_E2": add_2E2_graph,
    "3_E1": add_3E1_graph, "3_E2": add_3E2_graph
}
batch_scheduler = BatchScheduler(robots, elevators, stair_graph, elevator_graphs)
task_counter = 0


# 定义输入任务的Pydantic模型（单个任务）
class SingleTaskInput(BaseModel):
    task_id: int          # 任务ID
    skill: str            # 任务类型（如 "dog" 或 "human"）
    priority: int         # 任务优先级（如 "1", "2"）
    start_position: str   # 起点（如 "4_3_A"）
    target_position: str  # 终点（如 "6_3_G"）


# 定义批量输入模型（多个任务）
class BatchTaskInput(BaseModel):
    tasks: List[SingleTaskInput]  # 任务列表


class AssignmentOutput(BaseModel):
    task_id: int        # 任务ID
    robot_id: int       # 分配的机器人ID
    path: List[str]     # 预计路径（起点到终点的路径点列表）
    estimated_time: float # 预估完成时间（秒）


@app.get("/")
async def root():
    return {"message": "Robot Path API is alive!", "docs": "/docs"}


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