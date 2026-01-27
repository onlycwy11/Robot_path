"""
系统路由模块

提供系统状态、健康检查、重置等接口。
"""

from fastapi import APIRouter
import time

from api.schemas.system import HealthCheckResponse, SystemStatusResponse, ResetResponse
from api.state import global_state
from src.utils.config import DEFAULT_CONFIG

router = APIRouter(prefix="/system", tags=["System"])


@router.get("/system_status", response_model=SystemStatusResponse,summary="获取系统状态信息")
async def system_status():
    """
    获取系统详细状态信息，包括运行时间、机器人数量、MQTT 状态等。
    ```
    """
    uptime = global_state.get_uptime()

    # 格式化运行时间
    if uptime < 60:
        uptime_human = f"{uptime:.1f}s"
    elif uptime < 3600:
        minutes = int(uptime / 60)
        seconds = uptime % 60
        uptime_human = f"{minutes}m {seconds:.1f}s"
    else:
        hours = int(uptime / 3600)
        minutes = int((uptime % 3600) / 60)
        uptime_human = f"{hours}h {minutes}m"

    return SystemStatusResponse(
        schemaVersion="v1",
        messageType="system_status",
        initialized=global_state.initialized,
        campusName=global_state.campus_name,
        uptimeSeconds=uptime,
        uptimeHuman=uptime_human,
        robotCount=global_state.get_robot_count(),
        elevatorCount=global_state.get_elevator_count(),
        mqttConnected=global_state.mqtt_connected,
        mqttHost=DEFAULT_CONFIG.mqtt.host if DEFAULT_CONFIG.mqtt.enabled else None,
        mqttPort=DEFAULT_CONFIG.mqtt.port if DEFAULT_CONFIG.mqtt.enabled else None
    )

