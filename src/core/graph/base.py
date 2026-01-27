"""
Graph 基类

提供图的基础操作：添加节点、添加边、邻接表管理。
"""

from typing import Dict, List, Tuple, Set


class Graph:
    """
    有向加权图

    使用邻接表存储边：edges[u] = [(v, weight), ...]

    Attributes:
        edges: 邊邻接表
        nodes: 节点集合（自动维护）
    """

    def __init__(self):
        self.edges: Dict[str, List[Tuple[str, float]]] = {}
        self._nodes: Set[str] = set()

    @property
    def nodes(self) -> Set[str]:
        """获取所有节点集合"""
        return self._nodes

    def add_node(self, node: str) -> bool:
        """
        添加节点

        Args:
            node: 节点名称

        Returns:
            是否为新节点（True=新添加，False=已存在）
        """
        if node not in self._nodes:
            self._nodes.add(node)
            self.edges.setdefault(node, [])
            return True
        return False

    def add_edge(
        self,
        u: str,
        v: str,
        weight: float,
        bidirectional: bool = True
    ) -> None:
        """
        添加边

        Args:
            u: 起始节点
            v: 目标节点
            weight: 边权重（时间代价）
            bidirectional: 是否双向边
        """
        self.add_node(u)
        self.add_node(v)

        weight = float(weight)

        # 添加正向边
        self._add_single_edge(u, v, weight)

        # 添加反向边（如果是双向）
        if bidirectional:
            self._add_single_edge(v, u, weight)

    def _add_single_edge(self, u: str, v: str, weight: float) -> None:
        """添加单条边（内部方法）"""
        # 检查是否已存在相同的边
        for neighbor, w in self.edges.get(u, []):
            if neighbor == v and w == weight:
                return  # 已存在，不重复添加

        self.edges.setdefault(u, []).append((v, weight))

    def get_neighbors(self, node: str) -> List[Tuple[str, float]]:
        """
        获取节点的邻居列表

        Args:
            node: 节点名称

        Returns:
            邻居列表 [(neighbor, weight), ...]
        """
        return self.edges.get(node, [])

    def get_edge_weight(self, u: str, v: str) -> float:
        """
        获取边的权重

        Args:
            u: 起始节点
            v: 目标节点

        Returns:
            边权重，如果不存在则返回 inf
        """
        for neighbor, weight in self.edges.get(u, []):
            if neighbor == v:
                return weight
        return float("inf")

    def has_node(self, node: str) -> bool:
        """检查节点是否存在"""
        return node in self._nodes

    def has_edge(self, u: str, v: str) -> bool:
        """检查边是否存在"""
        for neighbor, _ in self.edges.get(u, []):
            if neighbor == v:
                return True
        return False

    def remove_node(self, node: str) -> bool:
        """
        移除节点及其所有相关边

        Args:
            node: 要移除的节点

        Returns:
            是否成功移除
        """
        if node not in self._nodes:
            return False

        # 移除该节点的所有出边
        self.edges.pop(node, None)
        self._nodes.remove(node)

        # 移除指向该节点的所有入边
        for u in list(self.edges.keys()):
            self.edges[u] = [(v, w) for v, w in self.edges[u] if v != node]

        return True

    def clear(self) -> None:
        """清空图"""
        self.edges.clear()
        self._nodes.clear()

    def copy(self) -> 'Graph':
        """创建图的深拷贝"""
        new_graph = Graph()
        new_graph._nodes = self._nodes.copy()
        new_graph.edges = {k: v.copy() for k, v in self.edges.items()}
        return new_graph

    def __len__(self) -> int:
        """节点数量"""
        return len(self._nodes)

    def __contains__(self, node: str) -> bool:
        """检查节点是否存在"""
        return node in self._nodes

    def __repr__(self) -> str:
        return f"Graph(nodes={len(self._nodes)}, edges={sum(len(e) for e in self.edges.values())})"