"""
路径分析器

提供路径分段分析、电梯节点提取等功能。
"""

from dataclasses import dataclass
from typing import List, Dict, Tuple, Optional, Any

from src.core.graph.base import Graph
from src.core.graph.dijkstra import DijkstraResult


@dataclass
class PathSegments:
    """
    路径分段信息

    用于两部电梯跨楼运输场景。
    """
    before: float = 0.0         # 到达第一部电梯前
    between_1: float = 0.0      # 第一部电梯运行
    transfer: float = 0.0       # 两部电梯间步行
    between_2: float = 0.0      # 第二部电梯运行
    after: float = 0.0          # 出电梯后到达目的地


class PathAnalyzer:
    """
    路径分析器

    分析路径结构，提取电梯节点，计算分段时间。
    """

    # 电梯标识符
    ELEVATOR_INDICATORS = ["_E1", "_E2", "_E3", "_E4", "_E5", "_E6", "_E"]

    # 楼梯标识符
    STAIR_INDICATORS = ["Stair1", "Stair2", "stair1", "stair2", "_Stair"]

    def __init__(self, graph: Graph):
        self.graph = graph

    def extract_elevator_nodes(
        self,
        path: List[str],
        indicator: str = "_E"
    ) -> List[str]:
        """
        从路径提取电梯节点

        Args:
            path: 节点路径
            indicator: 电梯标识

        Returns:
            电梯节点列表
        """
        elevator_nodes = []

        for i in range(len(path) - 1):
            current = path[i]
            next_node = path[i + 1]

            # 判断是否为电梯连接
            if self._is_elevator_connection(current, next_node, indicator):
                if current not in elevator_nodes:
                    elevator_nodes.append(current)
                if next_node not in elevator_nodes:
                    elevator_nodes.append(next_node)

        return elevator_nodes

    def extract_all_elevator_nodes(self, path: List[str]) -> List[str]:
        """
        从路径提取所有电梯节点（使用全部电梯标识符）

        Args:
            path: 节点路径

        Returns:
            电梯节点列表
        """
        elevator_nodes = []

        for node in path:
            if self._is_elevator_node(node):
                elevator_nodes.append(node)

        return elevator_nodes

    def calculate_segments_single_elevator(
        self,
        path: List[str],
        e1: str,
        e2: str
    ) -> Dict[str, float]:
        """
        计算单电梯路径的分段时间

        Args:
            path: 节点路径
            e1: 电梯起点
            e2: 电梯终点

        Returns:
            {"before": float, "between": float, "after": float}
        """
        before = 0.0
        between = 0.0
        after = 0.0

        if e1 not in path or e2 not in path:
            return {"before": 0.0, "between": 0.0, "after": 0.0}

        idx_e1 = path.index(e1)
        idx_e2 = path.index(e2)

        for i in range(len(path) - 1):
            u, v = path[i], path[i + 1]
            weight = self.graph.get_edge_weight(u, v)

            if i < idx_e1:
                before += weight
            elif idx_e1 <= i < idx_e2:
                between += weight
            else:
                after += weight

        return {"before": before, "between": between, "after": after}

    def calculate_segments_double_elevator(
        self,
        path: List[str],
        e1: str,
        e2: str,
        e3: str,
        e4: str
    ) -> PathSegments:
        """
        计算双电梯路径的分段时间（跨楼运输）

        Args:
            path: 节点路径
            e1: 第一部电梯起点
            e2: 第一部电梯终点
            e3: 第二部电梯起点
            e4: 第二部电梯终点

        Returns:
            PathSegments 分段信息
        """
        segments = PathSegments()

        indices = {}
        for node in [e1, e2, e3, e4]:
            if node in path:
                indices[node] = path.index(node)

        if len(indices) < 4:
            return segments

        idx_e1, idx_e2, idx_e3, idx_e4 = indices[e1], indices[e2], indices[e3], indices[e4]

        for i in range(len(path) - 1):
            u, v = path[i], path[i + 1]
            weight = self.graph.get_edge_weight(u, v)

            if i < idx_e1:
                segments.before += weight
            elif idx_e1 <= i < idx_e2:
                segments.between_1 += weight
            elif idx_e2 <= i < idx_e3:
                segments.transfer += weight
            elif idx_e3 <= i < idx_e4:
                segments.between_2 += weight
            else:
                segments.after += weight

        return segments

    def analyze_enhanced_path(
        self,
        result: Dict[str, Any],
        elevator_indicator: str = "_E"
    ) -> Dict[str, Any]:
        """
        分析增强版路径结果

        处理 dijkstra_extra_pro 的结果，自动判断电梯数量。

        Args:
            result: dijkstra_enhanced 返回的结果
            elevator_indicator: 电梯标识

        Returns:
            扩展后的路径分析结果
        """
        path = result.get("path", [])
        total_time = result.get("total_time", float("inf"))

        if not path:
            return result

        # 提取所有电梯节点
        elevator_nodes = self.extract_all_elevator_nodes(path)

        # 根据电梯节点数量决定分析方式
        if len(elevator_nodes) == 0:
            # 无电梯
            return {
                "path": path,
                "total_time": total_time,
                "segments": {"before": 0.0, "between_1": 0.0, "transfer": 0.0, "between_2": 0.0, "after": total_time},
                "elevator_nodes": [(None, None)]
            }

        elif len(elevator_nodes) == 2:
            # 单电梯（楼内）
            e1, e2 = elevator_nodes[0], elevator_nodes[1]
            segments = self.calculate_segments_single_elevator(path, e1, e2)
            return {
                "path": path,
                "total_time": total_time,
                "segments": {"before": segments["before"], "between_1": 0.0, "transfer": 0.0, "between_2": segments["between"], "after": segments["after"]},
                "elevator_nodes": [(e1, e2)]
            }

        elif len(elevator_nodes) == 4:
            # 双电梯（跨楼）
            e1, e2, e3, e4 = elevator_nodes[0], elevator_nodes[1], elevator_nodes[2], elevator_nodes[3]
            segments = self.calculate_segments_double_elevator(path, e1, e2, e3, e4)
            return {
                "path": path,
                "total_time": total_time,
                "segments": {
                    "before": segments.before,
                    "between_1": segments.between_1,
                    "transfer": segments.transfer,
                    "between_2": segments.between_2,
                    "after": segments.after
                },
                "elevator_nodes": [(e1, e2), (e3, e4)]
            }

        else:
            # 其他情况（返回默认）
            return {
                "path": path,
                "total_time": total_time,
                "segments": {"before": 0.0, "between_1": 0.0, "transfer": 0.0, "between_2": 0.0, "after": total_time},
                "elevator_nodes": [(None, None)]
            }

    def _is_elevator_node(self, node: str) -> bool:
        """判断节点是否为电梯节点"""
        return any(indicator in node for indicator in self.ELEVATOR_INDICATORS)

    def _is_elevator_connection(self, node1: str, node2: str, indicator: str) -> bool:
        """判断两个节点是否为电梯连接"""
        return indicator in node1 and indicator in node2

    def is_stair_node(self, node: str) -> bool:
        """判断节点是否为楼梯节点"""
        return any(indicator in node for indicator in self.STAIR_INDICATORS)

    def is_stair_connection(self, node1: str, node2: str) -> bool:
        """判断两个节点是否为楼梯连接"""
        return any(
            indicator in node1 and indicator in node2
            for indicator in self.STAIR_INDICATORS
        )

    def get_path_type(self, node1: str, node2: str) -> str:
        """
        判断两个节点之间的连接类型

        Args:
            node1: 第一个节点
            node2: 第二个节点

        Returns:
            "elevator", "stair", 或 "walk"
        """
        if self._is_elevator_connection(node1, node2, "_E"):
            return "elevator"
        elif self.is_stair_connection(node1, node2):
            return "stair"
        else:
            return "walk"