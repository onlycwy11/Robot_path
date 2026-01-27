"""
位置解析工具模块

提供位置节点解析、坐标提取等功能。
"""

from dataclasses import dataclass
from typing import Optional, Tuple

from src.utils.exceptions import InvalidPositionError


@dataclass(frozen=True)
class PositionInfo:
    """
    位置信息数据类

    Attributes:
        floor: 楼层号（支持半层如4.5）
        building: 楼号
        local_id: 局部标识（如房间号、电梯号等）
    """
    floor: float  # 改为 float 支持半层楼号
    building: str
    local_id: str

    def to_node_name(self) -> str:
        """转换为节点名称"""
        return f"{self.floor}_{self.building}_{self.local_id}"


def parse_position(node: str) -> PositionInfo:
    """
    解析位置节点名称

    节点命名格式: {floor}_{building}_{local_id}
    例如: "4_3_A" -> PositionInfo(floor=4, building="3", local_id="A")
         "4.5_1_p1" -> PositionInfo(floor=4.5, building="1", local_id="p1")

    Args:
        node: 位置节点名称

    Returns:
        PositionInfo 位置信息对象

    Raises:
        InvalidPositionError: 无效的位置格式
    """
    parts = node.split("_")
    if len(parts) < 2:
        raise InvalidPositionError(node)

    try:
        # 支持半层楼号如 "4.5"
        floor = float(parts[0])
    except ValueError:
        raise InvalidPositionError(node, f"Invalid floor number: {parts[0]}")

    building = parts[1]
    local_id = parts[2] if len(parts) > 2 else ""

    return PositionInfo(floor=floor, building=building, local_id=local_id)


def get_floor_from_node(node: str) -> float:
    """从节点名称提取楼层号（支持半层）"""
    return parse_position(node).floor


def get_building_from_node(node: str) -> str:
    """从节点名称提取楼号"""
    return parse_position(node).building


def needs_two_elevators(start: str, target: str) -> bool:
    """判断是否需要使用两部电梯（跨楼且起点终点都不在1层）"""
    s = parse_position(start)
    t = parse_position(target)
    return s.building != t.building and s.floor != 1 and t.floor != 1


def is_same_building(start: str, target: str) -> bool:
    """判断两点是否在同一栋楼"""
    return parse_position(start).building == parse_position(target).building


def is_same_floor(start: str, target: str) -> bool:
    """判断两点是否在同一楼层"""
    return parse_position(start).floor == parse_position(target).floor


def create_elevator_access_node(room_node: str, elevator_marker: str) -> str:
    """创建电梯访问节点名称：1_{building}_{elevator_code}"""
    try:
        building = get_building_from_node(room_node)
    except InvalidPositionError:
        building = "1"

    # 从电梯标记提取电梯编号
    if "_" in elevator_marker:
        parts = elevator_marker.split("_")
        elevator_code = parts[1] if len(parts) == 2 else "E1"
    else:
        elevator_code = elevator_marker

    return f"1_{building}_{elevator_code}"