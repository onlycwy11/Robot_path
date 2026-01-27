"""
路径工具函数模块

提供路径合并、路径显示等工具函数。
"""

from typing import List, Tuple, Optional, Dict, Any

from src.utils.exceptions import PathNotFoundError


def merge_paths(path1: List[str], path2: List[str]) -> List[str]:
    """合并两个路径，去除重复的中间节点"""
    if not path1 or not path2:
        return path1 + path2
    if path1[-1] != path2[0]:
        raise ValueError(f"Paths cannot merge: path1 ends at {path1[-1]}, but path2 starts at {path2[0]}")
    return path1[:-1] + path2


def has_stairs_in_path(path_info: Dict[str, Any]) -> bool:
    """判断路径是否包含楼梯"""
    if path_info.get("type") == "stair":
        return True
    return any("stair" in node.lower() for node in path_info.get("path", []))




def is_elevator_connection(node1: str, node2: str) -> bool:
    """
    判断两个节点之间是否为电梯连接

    Args:
        node1: 第一个节点
        node2: 第二个节点

    Returns:
        是否为电梯连接
    """
    elevator_indicators = ["_E1", "_E2", "_E3", "_E4", "_E5", "_E6"]
    return any(
        indicator in node1 and indicator in node2
        for indicator in elevator_indicators
    )


def is_stair_connection(node1: str, node2: str) -> bool:
    """
    判断两个节点之间是否为楼梯连接

    Args:
        node1: 第一个节点
        node2: 第二个节点

    Returns:
        是否为楼梯连接
    """
    stair_indicators = ["Stair1", "Stair2", "stair1", "stair2"]
    return any(
        indicator in node1 and indicator in node2
        for indicator in stair_indicators
    )


def segment_path_by_type(path: List[str]) -> List[Tuple[List[str], str]]:
    """
    按连接类型分段路径

    Args:
        path: 节点路径列表

    Returns:
        分段结果列表，每项为 (节点段, 类型)
    """
    if len(path) < 2:
        return [(path, "unknown")]

    segments = []
    current_segment = [path[0]]
    current_type = "walk"

    for i in range(len(path) - 1):
        node1, node2 = path[i], path[i + 1]

        if is_elevator_connection(node1, node2):
            segment_type = "elevator"
        elif is_stair_connection(node1, node2):
            segment_type = "stair"
        else:
            segment_type = "walk"

        if segment_type != current_type and len(current_segment) > 1:
            segments.append((current_segment, current_type))
            current_segment = [node1]
            current_type = segment_type

        current_segment.append(node2)

    segments.append((current_segment, current_type))
    return segments


def calculate_total_wait_time(path_info: Dict[str, Any]) -> float:
    """计算路径的总等待时间"""
    return path_info.get("wait_time_1", 0.0) + path_info.get("wait_time_2", 0.0)