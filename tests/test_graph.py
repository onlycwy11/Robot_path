"""
Graph 模块单元测试

测试 Dijkstra 算法、路径分析、节点类型判断等功能。
"""

import pytest

from src.core.graph import Graph, calculate_elevator_time_need
from src.core.graph.dijkstra import dijkstra, dijkstra_enhanced, DijkstraResult
from src.core.graph.path_analyzer import PathAnalyzer
from src.utils.constants import NodeType, get_node_type, is_elevator_node


class TestGraphBase:
    """Graph 基类测试"""

    def test_graph_initialization(self):
        """测试 Graph 初始化"""
        graph = Graph()
        graph.add_edge("A", "B", 10.0)
        graph.add_edge("B", "C", 15.0)

        assert "A" in graph.nodes
        assert "B" in graph.nodes
        assert "C" in graph.nodes

    def test_add_edge_bidirectional(self):
        """测试边的双向添加"""
        graph = Graph()
        graph.add_edge("X", "Y", 20.0, bidirectional=True)

        # 双向边都应该存在
        assert graph.has_edge("X", "Y")
        assert graph.has_edge("Y", "X")

    def test_add_edge_unidirectional(self):
        """测试单向边"""
        graph = Graph()
        graph.add_edge("X", "Y", 20.0, bidirectional=False)

        assert graph.has_edge("X", "Y")
        assert not graph.has_edge("Y", "X")


class TestDijkstraAlgorithm:
    """Dijkstra 算法测试"""

    def setup_method(self):
        """每个测试方法前的设置"""
        self.graph = Graph()
        # 创建测试图：A-B-C-D
        self.graph.add_edge("A", "B", 10.0, bidirectional=True)
        self.graph.add_edge("B", "C", 15.0, bidirectional=True)
        self.graph.add_edge("C", "D", 20.0, bidirectional=True)
        # 另一条路径：A-E-D（更短）
        self.graph.add_edge("A", "E", 15.0, bidirectional=True)
        self.graph.add_edge("E", "D", 10.0, bidirectional=True)

    def test_shortest_path_basic(self):
        """测试基本最短路径"""
        result = dijkstra(self.graph, "A", "D")

        # 应该选择 A-E-D 路径（总距离 25）而不是 A-B-C-D（总距离 45）
        assert result.path == ["A", "E", "D"]
        assert result.total_time == 25.0

    def test_path_to_self(self):
        """测试到自身的路径"""
        result = dijkstra(self.graph, "A", "A")

        assert result.path == ["A"]
        assert result.total_time == 0.0

    def test_unreachable_node(self):
        """测试不可达节点"""
        # 添加一个孤立节点
        self.graph.add_node("Z")

        result = dijkstra(self.graph, "A", "Z")

        assert result.path == []
        assert result.total_time == float('inf')

    def test_single_edge_path(self):
        """测试单边路径"""
        result = dijkstra(self.graph, "A", "B")

        assert result.path == ["A", "B"]
        assert result.total_time == 10.0

    def test_dijkstra_result_properties(self):
        """测试 DijkstraResult 属性"""
        result = dijkstra(self.graph, "A", "D")

        assert hasattr(result, 'path')
        assert hasattr(result, 'total_time')
        assert hasattr(result, 'reachable')

    def test_dijkstra_result_empty(self):
        """测试 DijkstraResult.empty()"""
        empty = DijkstraResult.empty()

        assert empty.path == []
        assert empty.total_time == float('inf')
        assert not empty.reachable


class TestPathAnalyzer:
    """路径分析器测试"""

    def setup_method(self):
        self.graph = Graph()
        self.graph.add_edge("4_1_p1", "4_1_E1", 5.0)
        self.graph.add_edge("4_1_E1", "5_1_E1", 10.0)
        self.graph.add_edge("5_1_E1", "5_1_p1", 5.0)
        self.analyzer = PathAnalyzer(self.graph)

    def test_extract_elevator_nodes(self):
        """测试提取电梯节点"""
        path = ["4_1_p1", "4_1_E1", "5_1_E1", "5_1_p1"]

        elevator_nodes = self.analyzer.extract_elevator_nodes(path)

        assert "4_1_E1" in elevator_nodes
        assert "5_1_E1" in elevator_nodes
        assert "4_1_p1" not in elevator_nodes

    def test_extract_all_elevator_nodes(self):
        """测试提取所有电梯节点"""
        path = ["4_1_p1", "4_1_E1", "5_1_E1", "5_1_p1"]

        elevator_nodes = self.analyzer.extract_all_elevator_nodes(path)

        assert len(elevator_nodes) == 2
        assert "4_1_E1" in elevator_nodes
        assert "5_1_E1" in elevator_nodes

    def test_empty_path(self):
        """测试空路径"""
        elevator_nodes = self.analyzer.extract_elevator_nodes([])

        assert elevator_nodes == []

    def test_is_stair_node(self):
        """测试楼梯节点判断"""
        assert self.analyzer.is_stair_node("4_1_Stair1")
        assert not self.analyzer.is_stair_node("4_1_p1")


class TestNodeType:
    """节点类型判断测试"""

    def test_elevator_node_detection(self):
        """测试电梯节点检测"""
        assert get_node_type("4_1_E1") == NodeType.ELEVATOR
        assert get_node_type("5_1_E2") == NodeType.ELEVATOR
        assert get_node_type("3_1_E") == NodeType.ELEVATOR

    def test_stair_node_detection(self):
        """测试楼梯节点检测"""
        assert get_node_type("4_1_Stair1") == NodeType.STAIR
        assert get_node_type("5_1_stair2") == NodeType.STAIR

    def test_room_node_detection(self):
        """测试房间节点检测"""
        assert get_node_type("4_1_p1") == NodeType.ROOM
        assert get_node_type("5_1_p7") == NodeType.ROOM
        assert get_node_type("A_B_C") == NodeType.ROOM  # 无特殊标识符

    def test_special_node_detection(self):
        """测试特殊节点检测"""
        assert get_node_type("charge_point") == NodeType.SPECIAL
        assert get_node_type("Charge_station") == NodeType.SPECIAL

    def test_is_elevator_node_function(self):
        """测试 is_elevator_node 函数"""
        assert is_elevator_node("4_1_E1") == True
        assert is_elevator_node("4_1_p1") == False
        assert is_elevator_node("4_1_Stair1") == False

    def test_node_type_methods(self):
        """测试 NodeType 方法"""
        assert NodeType.ELEVATOR.is_elevator() == True
        assert NodeType.STAIR.is_elevator() == False

        assert NodeType.STAIR.is_stair() == True
        assert NodeType.ROOM.is_stair() == False

        assert NodeType.ROOM.is_traversable() == True
        assert NodeType.ELEVATOR.is_traversable() == True


class TestCalculateElevatorTime:
    """电梯时间计算测试"""

    def test_basic_calculation(self):
        """测试基本计算"""
        # 公式: 1.5 + 1.5 + 1.75*n + 1.5
        # n=1: 6.25
        # n=3: 9.75
        assert calculate_elevator_time_need(1) == 6.25
        assert calculate_elevator_time_need(3) == 9.75

    def test_zero_floor(self):
        """测试零楼层差"""
        assert calculate_elevator_time_need(0) == 0.0


# pytest 配置
if __name__ == "__main__":
    pytest.main([__file__, "-v"])