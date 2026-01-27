"""
Dijkstra 最短路径算法

提供标准 Dijkstra 和增强版 Dijkstra（带路径分段分析）。
"""

import heapq
from dataclasses import dataclass
from typing import List, Tuple, Optional, Dict, Any

from src.core.graph.base import Graph
from src.utils.logger import graph_logger


@dataclass
class DijkstraResult:
    """
    Dijkstra 路径搜索结果

    Attributes:
        path: 节点路径列表
        total_time: 总时间代价
        reachable: 是否可达
    """
    path: List[str]
    total_time: float

    @property
    def reachable(self) -> bool:
        """是否可达"""
        return len(self.path) > 0 and self.total_time < float("inf")

    @classmethod
    def empty(cls) -> 'DijkstraResult':
        """创建空结果（不可达）"""
        return cls(path=[], total_time=float("inf"))

    @classmethod
    def create(cls, path: List[str], total_time: float) -> 'DijkstraResult':
        """创建成功结果"""
        return cls(path=path, total_time=total_time)


def dijkstra(graph: Graph, start: str, target: str) -> DijkstraResult:
    """
    标准 Dijkstra 最短路径搜索

    Args:
        graph: 图对象
        start: 起始节点
        target: 目标节点

    Returns:
        DijkstraResult 路径搜索结果
    """
    # 检查节点存在性
    if not graph.has_node(start):
        graph_logger.warning(f"Start node '{start}' not found in graph")
        return DijkstraResult.empty()

    if not graph.has_node(target):
        graph_logger.warning(f"Target node '{target}' not found in graph")
        return DijkstraResult.empty()

    # Dijkstra 搜索
    priority_queue: List[Tuple[float, str, List[str]]] = [(0.0, start, [])]
    visited: set = set()

    while priority_queue:
        cost, node, path = heapq.heappop(priority_queue)

        if node in visited:
            continue

        visited.add(node)
        path = path + [node]

        # 到达目标
        if node == target:
            return DijkstraResult.create(path, cost)

        # 扩展邻居
        for neighbor, weight in graph.get_neighbors(node):
            if neighbor not in visited:
                heapq.heappush(priority_queue, (cost + weight, neighbor, path))

    # 不可达
    graph_logger.debug(f"No path from '{start}' to '{target}'")
    return DijkstraResult.empty()


def dijkstra_enhanced(
    graph: Graph,
    start: str,
    target: str,
    elevator_indicator: str = "_E"
) -> Dict[str, Any]:
    """
    增强 Dijkstra：返回路径分段信息

    用于电梯路径分析，提取电梯节点并计算分段时间。

    Args:
        graph: 图对象
        start: 起始节点
        target: 目标节点
        elevator_indicator: 电梯节点标识（默认 "_E"）

    Returns:
        {
            "path": List[str],          # 节点路径
            "total_time": float,        # 总时间
            "segments": {                # 分段时间
                "before": float,        # 到达电梯前
                "between": float,       # 电梯运行中
                "after": float          # 出电梯后
            },
            "elevator_nodes": Tuple[str, str]  # (电梯起点, 电梯终点)
        }
    """
    # 先执行标准搜索
    result = dijkstra(graph, start, target)

    if not result.reachable:
        return {
            "path": [],
            "total_time": float("inf"),
            "segments": {"before": 0.0, "between": 0.0, "after": 0.0},
            "elevator_nodes": (None, None)
        }

    path = result.path
    total_time = result.total_time

    # 提取电梯节点（使用电梯标识而非硬编码 E1/E2）
    elevator_nodes = _extract_elevator_nodes(path, elevator_indicator)

    # 如果没有电梯节点或少于2个
    if len(elevator_nodes) < 2:
        e1 = elevator_nodes[0] if elevator_nodes else None
        return {
            "path": path,
            "total_time": total_time,
            "segments": {"before": 0.0, "between": 0.0, "after": total_time},
            "elevator_nodes": (e1, None)
        }

    # 计算分段时间
    e1, e2 = elevator_nodes[0], elevator_nodes[1]
    segments = _calculate_segments(graph, path, e1, e2)

    return {
        "path": path,
        "total_time": total_time,
        "segments": segments,
        "elevator_nodes": (e1, e2)
    }


def _extract_elevator_nodes(path: List[str], indicator: str) -> List[str]:
    """
    从路径中提取电梯节点

    Args:
        path: 节点路径
        indicator: 电梯标识（如 "_E"）

    Returns:
        电梯节点列表
    """
    elevator_nodes = []

    for i in range(len(path) - 1):
        current = path[i]
        next_node = path[i + 1]

        # 判断是否为电梯连接（相邻两个节点都包含电梯标识）
        if indicator in current and indicator in next_node:
            # 避免重复添加
            if current not in elevator_nodes:
                elevator_nodes.append(current)
            if next_node not in elevator_nodes:
                elevator_nodes.append(next_node)

    return elevator_nodes


def _calculate_segments(
    graph: Graph,
    path: List[str],
    e1: str,
    e2: str
) -> Dict[str, float]:
    """
    计算路径分段时间

    Args:
        graph: 图对象
        path: 节点路径
        e1: 第一个电梯节点
        e2: 第二个电梯节点

    Returns:
        {"before": float, "between": float, "after": float}
    """
    before = 0.0
    between = 0.0
    after = 0.0

    idx_e1 = path.index(e1)
    idx_e2 = path.index(e2)

    for i in range(len(path) - 1):
        u, v = path[i], path[i + 1]
        weight = graph.get_edge_weight(u, v)

        if i < idx_e1:
            before += weight
        elif idx_e1 <= i < idx_e2:
            between += weight
        else:
            after += weight

    return {"before": before, "between": between, "after": after}