"""
机器人路由模块

提供机器人状态查询接口，返回 MQTT 标准格式数据。
"""

from fastapi import APIRouter, HTTPException, Query
from typing import Optional, List
import time

from src.schedulers.batch_scheduler import get_robot_status_real_time

from api.schemas.robot import (
    RobotStatusAgg,
    RobotsStatusResponse,
    RobotInfo,
    StatusInfo,
    Position3D,
    LocationInfo,
    HeartbeatInfo,
)
from api.state import global_state

router = APIRouter(prefix="/robots", tags=["Robots"])


@router.get("/status", response_model=RobotsStatusResponse,summary="查询机器人状态")
async def get_robot_status(
    robot_id: Optional[int] = Query(
        None,
        description="机器人ID。不传则返回所有机器人状态；传入则返回单个机器人详细状态"
    ),
    include_tasks: bool = Query(
        False,
        description="是否包含当前任务信息"
    )
):
    """
    查询机器人状态
    """
    if not global_state.initialized:
        raise HTTPException(
            status_code=400,
            detail={"errorCode": "NOT_INITIALIZED", "errorMessage": "系统未初始化，请先调用 /map/initialize"}
        )

    scheduler = global_state.batch_scheduler
    mqtt_cache = global_state.mqtt_cache
    now = time.strftime("%Y-%m-%dT%H:%M:%S+08:00")

    # 确定查询范围
    if robot_id is not None:
        # 查询单个机器人
        robot_ids = [robot_id]
        # 检查机器人是否存在
        if not scheduler._find_robot_by_id(robot_id):
            raise HTTPException(
                status_code=404,
                detail={"errorCode": "ROBOT_NOT_FOUND", "errorMessage": f"机器人 {robot_id} 不存在"}
            )
    else:
        # 查询所有机器人
        robot_ids = [r.id for r in scheduler.robots]

    robots_status = []
    for rid in robot_ids:
        status_agg = _build_robot_status_agg(rid, mqtt_cache, scheduler, include_tasks)
        robots_status.append(status_agg)

    return RobotsStatusResponse(
        schemaVersion="v1",
        messageType="robots_status_agg",
        publishedAt=now,
        robots=robots_status,
        totalCount=len(robots_status),
        mqttConnected=global_state.mqtt_connected
    )


def _build_robot_status_agg(
    robot_id: int,
    mqtt_cache: dict,
    scheduler,
    include_tasks: bool
) -> RobotStatusAgg:
    """
    构建机器人状态聚合消息

    优先从 MQTT 获取数据，无数据时字段为空
    """
    now = time.strftime("%Y-%m-%dT%H:%M:%S+08:00")
    robot = scheduler._find_robot_by_id(robot_id)
    mqtt_msg = mqtt_cache.get(robot_id)

    # 基本信息优先从 MQTT
    robot_info = RobotInfo(
        id=robot_id,
        sn=mqtt_msg.robot_sn if mqtt_msg else None,
        name=mqtt_msg.robot_name if mqtt_msg else (f"Dog{robot_id}" if robot and robot.skill == "dog" else f"Human{robot_id}"),
        brand=mqtt_msg.robot_brand if mqtt_msg else None,
        model=mqtt_msg.robot_model if mqtt_msg else None
    )

    # 状态信息优先从 MQTT
    status_info = None
    if mqtt_msg:
        status_info = StatusInfo(
            onlineStatus=mqtt_msg.online_status,
            taskStatus=mqtt_msg.task_status,
            robotState=mqtt_msg.robot_state,
            battery=mqtt_msg.battery,
            isCharging=mqtt_msg.is_charging,
            isMoving=mqtt_msg.is_moving,
            temperature=mqtt_msg.temperature
        )
    elif robot:
        # 无 MQTT 数据，从 Robot 对象获取部分信息
        status_info = StatusInfo(
            onlineStatus="unknown",
            taskStatus="idle" if robot.is_available() else "busy",
            robotState="idle" if robot.is_available() else "working",
            battery=int(robot.charge),
            isCharging=robot.is_charging,
            isMoving=None,  # 无法确定
            temperature=None
        )

    # 位置信息优先从 MQTT
    location_info = None
    if mqtt_msg:
        location_info = LocationInfo(
            mapName=mqtt_msg.map_name,
            position=Position3D(
                x=mqtt_msg.position_x,
                y=mqtt_msg.position_y,
                z=mqtt_msg.position_z,
                yaw=mqtt_msg.position_yaw
            )
        )
    elif robot:
        # 无 MQTT 数据，从 Robot 对象获取位置
        pos = robot.current_position
        if pos:
            location_info = LocationInfo(
                mapName=None,
                position=Position3D(x=pos[0], y=pos[1], z=pos[2], yaw=None)
            )

    # 心跳信息仅从 MQTT
    heartbeat_info = None
    if mqtt_msg:
        heartbeat_info = HeartbeatInfo(
            sourceTime=mqtt_msg.source_time,
            lastStateAt=mqtt_msg.last_state_at,
            stateAgeMs=mqtt_msg.state_age_ms,
            isStale=mqtt_msg.is_stale
        )

    return RobotStatusAgg(
        schemaVersion="v1",
        messageType="robot_status_agg",
        publishedAt=now,
        heartbeatSeq=mqtt_msg.heartbeat_seq if mqtt_msg else None,
        intervalSec=mqtt_msg.interval_sec if mqtt_msg else None,
        robot=robot_info,
        workspace=None,  # 工作空间信息暂不填充
        platform=None,   # 平台信息暂不填充
        status=status_info,
        location=location_info,
        heartbeat=heartbeat_info
    )


@router.get("/{robot_id}/tasks",summary="获取机器人任务列表")
async def get_robot_tasks(
    robot_id: int,
    include_path: bool = Query(True, description="是否包含路径详情")
):
    """
    获取机器人的任务列表
    """
    if not global_state.initialized:
        raise HTTPException(
            status_code=400,
            detail={"errorCode": "NOT_INITIALIZED", "errorMessage": "系统未初始化"}
        )

    scheduler = global_state.batch_scheduler
    robot = scheduler._find_robot_by_id(robot_id)

    if not robot:
        raise HTTPException(
            status_code=404,
            detail={"errorCode": "ROBOT_NOT_FOUND", "errorMessage": f"机器人 {robot_id} 不存在"}
        )

    tasks = []
    for task_id, task_info in robot.task_list.items():
        task_data = {
            "taskId": task_id,
            "startTime": task_info.start_time,
            "finishTime": task_info.finish_time,
            "pathLength": len(task_info.path)
        }
        if include_path:
            task_data["path"] = task_info.path
        tasks.append(task_data)

    return {
        "schemaVersion": "v1",
        "messageType": "robot_tasks_response",
        "robotId": robot_id,
        "tasks": tasks,
        "total": len(tasks)
    }
