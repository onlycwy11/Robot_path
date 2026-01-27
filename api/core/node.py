"""
API 核心辅助模块

提供 API 层使用的辅助函数。
"""

from pathlib import Path
import yaml
from typing import List, Dict, Any

from api.schemas.map import NodeInfo, ElevatorInfo
from src.utils.logger import graph_logger


def get_nodes_from_yaml(campus_name: str) -> List[NodeInfo]:
    """
    从 merged_nodes.yaml 获取节点列表

    Args:
        campus_name: 校园名称

    Returns:
        节点信息列表
    """
    base_path = Path(__file__).parent.parent.parent / "src" / "core" / "data"
    yaml_path = base_path / campus_name / "merged_nodes.yaml"

    if not yaml_path.exists():
        return []

    with open(yaml_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    node_list = data.get("node_list", [])
    nodes = []

    for n in node_list:
        name = n.get("node_name", "")
        if not name:
            continue

        # 解析节点名称：格式 floor_building_localId
        parts = name.split("_")
        # 楼层可能包含半层如 "4.5"
        try:
            floor = float(parts[0]) if len(parts) >= 1 else 0.0
        except ValueError:
            floor = 0.0
        try:
            building = int(parts[1]) if len(parts) >= 2 else 0
        except ValueError:
            building = 0

        # 获取坐标
        coord = n.get("node_coordinate", {}) or {}
        coordinates = {
            "x": coord.get("x", 0.0) * 0.05 * 100,
            "y": coord.get("y", 0.0) * 0.05 * 100,
            "z": coord.get("z", 0.0) * 0.05 * 100
        }

        # 判断节点类型
        local_id = parts[2] if len(parts) >= 3 else ""
        if "E" in local_id:
            node_type = "elevator"
        elif "Stair" in local_id or "stair" in local_id.lower():
            node_type = "stair"
        else:
            node_type = "room"

        nodes.append(NodeInfo(
            nodeName=name,
            floor=floor,
            building=building,
            localId=local_id,
            nodeType=node_type,
            coordinates=coordinates
        ))

    return nodes


def get_elevator_info_list(elevator_graphs: dict) -> List[ElevatorInfo]:
    """
    获取电梯信息列表

    Args:
        elevator_graphs: 电梯增强图字典

    Returns:
        电梯信息列表
    """
    elevators = []

    for elevator_id in elevator_graphs.keys():
        # 解析电梯ID：格式 building_Enumber
        parts = elevator_id.split("_")
        building = int(parts[0]) if len(parts) >= 1 else 0
        local_id = parts[1] if len(parts) >= 2 else ""

        elevators.append(ElevatorInfo(
            elevatorId=elevator_id,
            building=building,
            localId=local_id,
            currentFloor=1,
            scheduleCount=0
        ))

    return elevators


def validate_node_exists(node_name: str, campus_name: str) -> bool:
    """
    验证节点是否存在

    Args:
        node_name: 节点名称
        campus_name: 校园名称

    Returns:
        是否存在
    """
    nodes = get_nodes_from_yaml(campus_name)
    return any(n.nodeName == node_name for n in nodes)


def get_node_by_name(node_name: str, campus_name: str) -> NodeInfo | None:
    """
    根据名称获取节点信息

    Args:
        node_name: 节点名称
        campus_name: 校园名称

    Returns:
        节点信息或 None
    """
    nodes = get_nodes_from_yaml(campus_name)
    for n in nodes:
        if n.nodeName == node_name:
            return n
    return None