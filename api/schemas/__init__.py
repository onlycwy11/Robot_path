"""
API Schema 模块

集中管理所有 Pydantic 模型定义。
"""

from api.schemas.task import (
    SingleTaskInput,
    BatchTaskInput,
    AssignmentOutput,
    TaskCancelResponse,
)
from api.schemas.robot import (
    SkillEnum,
    RobotConfigInput,
    RobotsConfigInput,
    RobotInfo,
    WorkspaceInfo,
    PlatformInfo,
    StatusInfo,
    Position3D,
    LocationInfo,
    HeartbeatInfo,
    RobotStatusAgg,
    RobotsStatusResponse,
)
from api.schemas.map import (
    MapInitializeRequest,
    MapInitializeResponse,
    NodeInfo,
    NodesListResponse,
    ElevatorInfo,
    ElevatorsListResponse,
    MapConfig,
)
from api.schemas.system import (
    HealthCheckResponse,
    SystemStatusResponse,
    ResetResponse,
    RootResponse,
    ErrorResponse,
)

__all__ = [
    # Task schemas
    "SingleTaskInput",
    "BatchTaskInput",
    "AssignmentOutput",
    "TaskCancelResponse",
    # Robot schemas (input)
    "SkillEnum",
    "RobotConfigInput",
    "RobotsConfigInput",
    # Robot schemas (output - MQTT format)
    "RobotInfo",
    "WorkspaceInfo",
    "PlatformInfo",
    "StatusInfo",
    "Position3D",
    "LocationInfo",
    "HeartbeatInfo",
    "RobotStatusAgg",
    "RobotsStatusResponse",
    # Map schemas
    "MapInitializeRequest",
    "MapInitializeResponse",
    "NodeInfo",
    "NodesListResponse",
    "ElevatorInfo",
    "ElevatorsListResponse",
    "MapConfig",
    # System schemas
    "HealthCheckResponse",
    "SystemStatusResponse",
    "ResetResponse",
    "RootResponse",
    "ErrorResponse",
]