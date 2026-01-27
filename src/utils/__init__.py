# src/utils/__init__.py
"""
工具函数模块

包含:
- config: 系统配置参数
- constants: 常量定义
- path_utils: 路径合并等工具
- position_utils: 位置解析
- elevator_time: 电梯时间计算
- exceptions: 自定义异常
- mqtt_handler: MQTT 消息处理
"""

from src.utils.config import (
    SystemConfig,
    MQTTConfig,
    DEFAULT_CONFIG,
    calculate_elevator_time_need,
    load_config_from_env,
    load_mqtt_config_from_env,
    ELEVATOR_TIME_TABLE,
    get_elevator_time_from_table,
)
from src.utils.constants import (
    PATH_PHASE_KEYS,
    PathPhase,
    NodeType,
    RobotType,
    RobotStatus,
    PathType,
    ConflictStrategy,
    SPEED_LAND_CM,
    SPEED_STAIR_CM,
    SPEED_ELEVATOR_CM,
    # 节点类型判断函数
    get_node_type,
    is_elevator_node,
    is_stair_node,
)
from src.utils.position_utils import (
    PositionInfo,
    parse_position,
    get_floor_from_node,
    get_building_from_node,
    needs_two_elevators,
    is_same_building,
    is_same_floor,
    create_elevator_access_node,
)
from src.utils.path_utils import (
    merge_paths,
    has_stairs_in_path,
    is_elevator_connection,
    is_stair_connection,
    calculate_total_wait_time,
)
from src.utils.exceptions import (
    PathNotFoundError,
    ElevatorUnavailableError,
    NoFeasibleRobotError,
    RobotChargeLowError,
    ConflictResolutionError,
    InvalidPositionError,
)
from src.utils.mqtt_handler import (
    RobotStatusMessage,
    MQTTStatusHandler,
    create_mqtt_handler,
)
from src.utils.protocols import (
    PathFinder,
    Scheduler,
    ElevatorManager,
    RobotStateProvider,
)

__all__ = [
    "SystemConfig",
    "MQTTConfig",
    "DEFAULT_CONFIG",
    "calculate_elevator_time_need",
    "load_config_from_env",
    "load_mqtt_config_from_env",
    "ELEVATOR_TIME_TABLE",
    "get_elevator_time_from_table",
    "PATH_PHASE_KEYS",
    "PathPhase",
    "NodeType",
    "RobotType",
    "RobotStatus",
    "PathType",
    "ConflictStrategy",
    "SPEED_LAND_CM",
    "SPEED_STAIR_CM",
    "SPEED_ELEVATOR_CM",
    # 节点类型判断函数
    "get_node_type",
    "is_elevator_node",
    "is_stair_node",
    "PositionInfo",
    "parse_position",
    "get_floor_from_node",
    "get_building_from_node",
    "needs_two_elevators",
    "is_same_building",
    "is_same_floor",
    "create_elevator_access_node",
    "merge_paths",
    "has_stairs_in_path",
    "is_elevator_connection",
    "is_stair_connection",
    "calculate_total_wait_time",
    "PathNotFoundError",
    "ElevatorUnavailableError",
    "NoFeasibleRobotError",
    "RobotChargeLowError",
    "ConflictResolutionError",
    "InvalidPositionError",
    "RobotStatusMessage",
    "MQTTStatusHandler",
    "create_mqtt_handler",
    # Protocol 接口
    "PathFinder",
    "Scheduler",
    "ElevatorManager",
    "RobotStateProvider",
]