"""
常量定义模块

定义系统中使用的常量和枚举。
"""

from __future__ import annotations

from enum import Enum
from typing import Dict


class PathPhase(Enum):
    """路径阶段枚举"""
    PICK = 0       # 取货阶段
    DELIVER = 1    # 送货阶段

    def get_info_key(self) -> str:
        """获取对应的 path_info 字典键"""
        return PATH_PHASE_KEYS[self.value]


# 路径阶段到 path_info 字典键的映射
PATH_PHASE_KEYS: Dict[int, str] = {
    0: "pick_path_info",
    1: "deliver_path_info",
}


class NodeType(Enum):
    """
    节点类型枚举

    用于统一节点类型判断，替代硬编码的 "E1"/"E2" 检查。
    """
    ROOM = "room"         # 房间节点 (如 p1, p2, p3)
    ELEVATOR = "elevator" # 电梯节点 (包含 _E 标识符)
    STAIR = "stair"       # 楼梯节点 (包含 Stair 标识符)
    SPECIAL = "special"   # 特殊节点 (充电点等)

    def is_elevator(self) -> bool:
        """是否为电梯节点"""
        return self == NodeType.ELEVATOR

    def is_stair(self) -> bool:
        """是否为楼梯节点"""
        return self == NodeType.STAIR

    def is_traversable(self) -> bool:
        """是否可通行"""
        return self in (NodeType.ROOM, NodeType.ELEVATOR, NodeType.STAIR)


# 节点标识符常量（用于类型判断）
NODE_ELEVATOR_INDICATORS = ["_E1", "_E2", "_E3", "_E4", "_E5", "_E6", "_E"]
NODE_STAIR_INDICATORS = ["Stair1", "Stair2", "stair1", "stair2", "_Stair"]
NODE_CHARGING_INDICATORS = ["charge", "Charge", "CHARGE"]


def get_node_type(node_name: str) -> NodeType:
    """
    根据节点名称判断节点类型

    Args:
        node_name: 节点名称（如 "4_1_p1", "4_1_E1", "4_1_Stair1"）
    """
    for indicator in NODE_ELEVATOR_INDICATORS:
        if indicator in node_name:
            return NodeType.ELEVATOR

    for indicator in NODE_STAIR_INDICATORS:
        if indicator in node_name:
            return NodeType.STAIR

    for indicator in NODE_CHARGING_INDICATORS:
        if indicator in node_name:
            return NodeType.SPECIAL

    return NodeType.ROOM


def is_elevator_node(node_name: str) -> bool:
    """判断是否为电梯节点"""
    return any(ind in node_name for ind in NODE_ELEVATOR_INDICATORS)


def is_stair_node(node_name: str) -> bool:
    """判断是否为楼梯节点"""
    return any(ind in node_name for ind in NODE_STAIR_INDICATORS)


class RobotType(Enum):
    """机器人类型枚举"""
    DOG = "dog"
    HUMAN = "human"


class RobotStatus(Enum):
    """机器人状态枚举"""
    IDLE = 0       # 空闲
    WORKING = 1    # 工作中


class PathType(Enum):
    """路径类型枚举"""
    STAIR = "stair"       # 楼梯路径
    ELEVATOR = "elevator"  # 电梯路径


class ConflictStrategy(Enum):
    """冲突解决策略枚举"""
    MAX_UTILIZATION = "strategy1"  # 策略1: 最大利用率
    MIN_COST = "strategy2"         # 策略2: 最小代价增量


# 速度常量 (cm/s)
SPEED_LAND_CM = 150.0   # 平地: 1.5 m/s = 150 cm/s
SPEED_STAIR_CM = 50.0   # 楼梯: 0.5 m/s = 50 cm/s
SPEED_ELEVATOR_CM = 200.0  # 电梯: 200 cm/s