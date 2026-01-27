"""
Graph 模块

提供图结构和路径搜索功能。
"""

from typing import Dict, Any, List, Tuple

from src.core.graph.base import Graph
from src.core.graph.dijkstra import DijkstraResult, dijkstra, dijkstra_enhanced
from src.core.graph.path_analyzer import PathAnalyzer, PathSegments


def calculate_elevator_time_need(delta_floor: int) -> float:
    """
    电梯跨层耗时计算（向后兼容）

    Args:
        delta_floor: 跨层数量

    Returns:
        电梯总耗时（秒）
    """
    from src.utils.config import DEFAULT_CONFIG
    return DEFAULT_CONFIG.elevator_time_for_floors(delta_floor)


# 向后兼容：为 Graph 类添加 dijkstra 相关方法
def _dijkstra_method(self, start: str, target: str) -> Tuple[List[str], float]:
    """标准 dijkstra 方法，返回 (path, cost)"""
    result = dijkstra(self, start, target)
    return (result.path, result.total_time)


def _dijkstra_extra(self, start: str, target: str) -> Dict[str, Any]:
    """向后兼容的 dijkstra_extra 方法"""
    return dijkstra_enhanced(self, start, target)


def _dijkstra_extra_pro(self, start: str, target: str) -> Dict[str, Any]:
    """向后兼容的 dijkstra_extra_pro 方法"""
    result = dijkstra_enhanced(self, start, target)
    analyzer = PathAnalyzer(self)
    return analyzer.analyze_enhanced_path(result)


# 动态添加方法到 Graph 类
Graph.dijkstra = _dijkstra_method
Graph.dijkstra_extra = _dijkstra_extra
Graph.dijkstra_extra_pro = _dijkstra_extra_pro


__all__ = [
    "Graph",
    "DijkstraResult",
    "dijkstra",
    "dijkstra_enhanced",
    "PathAnalyzer",
    "PathSegments",
    "calculate_elevator_time_need",
]