"""
Graph 模块（兼容层）

保持向后兼容性，导入重构后的模块。
原有的 calculate_elevator_time_need 函数保留。
"""

from typing import Dict, Any

from src.core.graph.base import Graph
from src.core.graph.dijkstra import dijkstra, dijkstra_enhanced, DijkstraResult
from src.core.graph.path_analyzer import PathAnalyzer, PathSegments


def calculate_elevator_time_need(delta_floor: int) -> float:
    """
    电梯跨层耗时计算（向后兼容）

    公式: 开门 + 关门 + 运行 + 开门 = 1.5 + 1.5 + 1.75 * n + 1.5
    """
    from src.utils.config import DEFAULT_CONFIG
    return DEFAULT_CONFIG.elevator_time_for_floors(delta_floor)


# 向后兼容：为 Graph 类添加 dijkstra_extra 和 dijkstra_extra_pro 方法
def _dijkstra_extra_compat(self, start: str, target: str) -> Dict[str, Any]:
    """向后兼容的 dijkstra_extra 方法"""
    return dijkstra_enhanced(self, start, target)


def _dijkstra_extra_pro_compat(self, start: str, target: str) -> Dict[str, Any]:
    """向后兼容的 dijkstra_extra_pro 方法"""
    result = dijkstra_enhanced(self, start, target)
    analyzer = PathAnalyzer(self)
    return analyzer.analyze_enhanced_path(result)


# 动态添加方法到 Graph 类
Graph.dijkstra = lambda self, s, t: dijkstra(self, s, t)
Graph.dijkstra_extra = _dijkstra_extra_compat
Graph.dijkstra_extra_pro = _dijkstra_extra_pro_compat


__all__ = [
    "Graph",
    "calculate_elevator_time_need",
    "dijkstra",
    "dijkstra_enhanced",
    "DijkstraResult",
    "PathAnalyzer",
    "PathSegments",
]