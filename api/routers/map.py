"""
地图路由模块

提供地图初始化、节点查询、电梯查询等接口。
"""

from fastapi import APIRouter, HTTPException, UploadFile, File, Form, Query
from typing import Optional
import json
import os
import shutil
from pathlib import Path
import time

from src.core.graph_loader import load_stair_graph_and_elevator_graphs
from src.core.template_to_topo import building_graph
from src.models.robot import Robot
from src.models.elevator import init_elevators
from src.schedulers.batch_scheduler import BatchScheduler
from src.utils.config import DEFAULT_CONFIG

from api.schemas.map import (
    MapInitializeResponse,
    NodesListResponse,
    NodeInfo,
    ElevatorsListResponse,
    ElevatorInfo,
)
from api.schemas.robot import RobotsConfigInput
from api.state import global_state
from api.core.node import get_nodes_from_yaml, get_elevator_info_list

router = APIRouter(prefix="/map", tags=["Map"])

BASE_PATH = Path(__file__).parent.parent.parent / "src" / "core" / "data"


@router.post("/initialize", response_model=MapInitializeResponse,summary="初始化系统，传入地图及机器人")
async def initialize(
    campus_name: str = Form(
        ...,
        description="校园/区域名称，用于标识和存储地图数据",
        examples=["test_campus", "sandun"]
    ),
    robots: str = Form(
        ...,
        description="""机器人配置 JSON 字符串。

格式示例：
```json
{
  "robots": [
    {"rid": 0, "skill": "dog", "position": "4_1_p1", "campus_name": "test_campus"},
    {"rid": 1, "skill": "human", "position": "5_1_p1", "campus_name": "test_campus"}
  ]
}
```
"""
    ),
    yaml_files: list[UploadFile] = File(
        ...,
        description="拓扑 YAML 文件列表。需要上传楼层的拓扑文件，如 floor_1_topo.yaml、stairs_1_topo.yaml 等"
    ),
    use_mqtt: bool = Form(
        True,
        description="默认启用 MQTT 实时状态"
    )
):
    """
    初始化地图和调度器
    """
    try:
        # 解析机器人配置
        robots_data = json.loads(robots)
        robots_config = RobotsConfigInput(**robots_data)

        # 创建校园数据目录
        yaml_dir = BASE_PATH / campus_name / "yaml"
        yaml_dir.mkdir(parents=True, exist_ok=True)

        # 保存上传的 YAML 文件
        for yaml_file in yaml_files:
            file_path = yaml_dir / yaml_file.filename
            with open(file_path, "wb") as buffer:
                shutil.copyfileobj(yaml_file.file, buffer)

        # 构建拓扑图
        building_graph(campus_name)

        # 加载图数据
        merged_yaml_path = BASE_PATH / campus_name / "merged_nodes.yaml"
        stair_graph, elevator_graphs = load_stair_graph_and_elevator_graphs(
            merged_nodes_yaml=str(merged_yaml_path),
            elevator_xy_tol=0.25,
            ensure_legacy_keys=True,
            speed_land=1.5,
            speed_stair=1.0
        )

        # 初始化电梯
        elevator_names = list(elevator_graphs.keys())
        elevators = init_elevators(elevator_names)

        # 创建机器人列表
        robot_list = []
        for r in robots_config.robots:
            robot_list.append(
                Robot(
                    rid=r.rid,
                    skill=r.skill.value,
                    position=r.position,
                    campus_name=r.campus_name,
                    config=DEFAULT_CONFIG
                )
            )

        # 创建调度器
        batch_scheduler = BatchScheduler(
            robot_list, elevators, stair_graph,
            elevator_graphs, campus_name
        )

        # 设置全局状态
        global_state.set_scheduler(
            batch_scheduler, stair_graph, elevator_graphs, campus_name
        )

        # 启动 MQTT（如果请求）
        mqtt_connected = False
        if use_mqtt:
            global_state.start_mqtt(config=DEFAULT_CONFIG)
            mqtt_connected = global_state.mqtt_connected

        # 获取节点数量
        nodes = get_nodes_from_yaml(campus_name)
        node_count = len(nodes)

        now = time.strftime("%Y-%m-%dT%H:%M:%S+08:00")

        return MapInitializeResponse(
            schemaVersion="v1",
            messageType="map_initialize_response",
            publishedAt=now,
            status="success",
            message="Map and scheduler initialized successfully",
            campusName=campus_name,
            elevatorCount=len(elevators),
            robotCount=len(robot_list),
            nodeCount=node_count,
            mqttConnected=mqtt_connected
        )

    except json.JSONDecodeError as e:
        raise HTTPException(
            status_code=400,
            detail={
                "errorCode": "INVALID_JSON",
                "errorMessage": f"机器人配置 JSON 解析失败: {str(e)}",
                "details": {"hint": "请确保 robots 参数是有效的 JSON 字符串"}
            }
        )
    except Exception as e:
        raise HTTPException(
            status_code=400,
            detail={
                "errorCode": "INITIALIZATION_FAILED",
                "errorMessage": str(e)
            }
        )


@router.get("/nodes", response_model=NodesListResponse,summary="获取地图节点列表")
async def get_nodes(
    floor: Optional[float] = Query(None, description="筛选指定楼层的节点，如 4 或 4.5"),
    node_type: Optional[str] = Query(None, description="筛选节点类型：room/elevator/stair"),
    limit: int = Query(100, ge=1, le=500, description="返回节点数量限制")
):
    """
    获取地图节点列表
    """
    if not global_state.initialized:
        raise HTTPException(
            status_code=400,
            detail={"errorCode": "NOT_INITIALIZED", "errorMessage": "系统未初始化"}
        )

    try:
        nodes = get_nodes_from_yaml(global_state.campus_name)

        # 筛选
        if floor is not None:
            nodes = [n for n in nodes if n.floor == floor]
        if node_type:
            nodes = [n for n in nodes if n.nodeType == node_type]

        # 限制数量
        nodes = nodes[:limit]

        now = time.strftime("%Y-%m-%dT%H:%M:%S+08:00")

        return NodesListResponse(
            schemaVersion="v1",
            messageType="nodes_list_response",
            publishedAt=now,
            campusName=global_state.campus_name,
            nodes=nodes,
            totalCount=len(nodes)
        )

    except Exception as e:
        raise HTTPException(
            status_code=400,
            detail={"errorCode": "NODES_QUERY_FAILED", "errorMessage": str(e)}
        )


@router.get("/elevators", response_model=ElevatorsListResponse,summary="获取地图电梯列表")
async def get_elevators():
    """
    获取电梯列表
    """
    if not global_state.initialized:
        raise HTTPException(
            status_code=400,
            detail={"errorCode": "NOT_INITIALIZED", "errorMessage": "系统未初始化"}
        )

    try:
        elevators = get_elevator_info_list(global_state.elevator_graphs)

        # 如果调度器存在，获取预约数量
        scheduler = global_state.batch_scheduler
        if scheduler:
            for elev in elevators:
                elevator_obj = scheduler.elevators.get(elev.elevatorId)
                if elevator_obj:
                    elev.scheduleCount = len(elevator_obj.schedule)

        now = time.strftime("%Y-%m-%dT%H:%M:%S+08:00")

        return ElevatorsListResponse(
            schemaVersion="v1",
            messageType="elevators_list_response",
            publishedAt=now,
            elevators=elevators,
            totalCount=len(elevators)
        )

    except Exception as e:
        raise HTTPException(
            status_code=400,
            detail={"errorCode": "ELEVATORS_QUERY_FAILED", "errorMessage": str(e)}
        )


@router.get("/info",summary="获取地图综合信息")
async def get_map_info():
    """
    获取地图综合信息

    返回节点、电梯、机器人等综合信息
    """
    if not global_state.initialized:
        raise HTTPException(
            status_code=400,
            detail={"errorCode": "NOT_INITIALIZED", "errorMessage": "系统未初始化"}
        )

    nodes = get_nodes_from_yaml(global_state.campus_name)
    elevators = get_elevator_info_list(global_state.elevator_graphs)

    return {
        "schemaVersion": "v1",
        "messageType": "map_info_response",
        "campusName": global_state.campus_name,
        "nodeCount": len(nodes),
        "elevatorCount": len(elevators),
        "robotCount": global_state.get_robot_count(),
        "floorList": sorted(set(n.floor for n in nodes)),
        "buildingList": sorted(set(n.building for n in nodes)),
        "nodeTypes": {
            "room": len([n for n in nodes if n.nodeType == "room"]),
            "elevator": len([n for n in nodes if n.nodeType == "elevator"]),
            "stair": len([n for n in nodes if n.nodeType == "stair"])
        }
    }