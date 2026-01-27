'''
Author: slighty-white 5273495@qq.com
Date: 2026-01-25 20:33:15
LastEditors: slighty-white 5273495@qq.com
LastEditTime: 2026-01-27 14:41:52
FilePath: /Robot_Path/Robot_path-main/src/core/graph.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''
# src/core/graph.py
import heapq
from typing import Dict, List, Tuple, Optional, Any
import math
from distutils.command.build_scripts import first_line_re

import matplotlib.pyplot as plt
import networkx as nx


class Graph:
    """
    最小 Graph：满足 batch_scheduler 使用
    - edges[u] = [(v, weight), ...]
    - dijkstra(start, end) -> (path, cost)
    - dijkstra_extra(start, end) -> dict（兼容 batch_scheduler 期望字段）
    """
    def __init__(self):
        self.edges: Dict[str, List[Tuple[str, float]]] = {}

    def add_node(self, node: str):
        if node not in self.edges:
            self.edges[node] = []

    def add_edge(self, u: str, v: str, w: float, bidirectional: bool = True):
        self.add_node(u)
        self.add_node(v)
        self.edges.setdefault(u, []).append((v, float(w)))
        if bidirectional:
            self.edges.setdefault(v, []).append((u, float(w)))

    def dijkstra(self, start: str, end: str):
        """Simple Dijkstra shortest path."""
        import heapq
        pq = [(0, start, [])]
        visited = set()
        while pq:
            cost, node, path = heapq.heappop(pq)
            if node in visited:
                continue
            visited.add(node)
            path = path + [node]
            if node == end:
                return path, cost
            for neighbor, weight in self.edges.get(node, []):
                if neighbor not in visited:
                    heapq.heappush(pq, (cost + weight, neighbor, path))
        return [], float("inf")

    def dijkstra_extra(self, start: str, end: str):
        """Enhanced Dijkstra: returns full path, total time, and safe segment times before/between/after E nodes."""
        import heapq

        def get_edge_weight(u, v):
            """Return weight of edge u->v, raise if not found."""
            for nb, w in self.edges.get(u, []):
                if nb == v:
                    return w
            raise ValueError(f"Edge weight not found for {u} -> {v}")

        pq = [(0, start, [])]
        visited = set()

        while pq:
            cost, node, path = heapq.heappop(pq)
            if node in visited:
                continue
            visited.add(node)
            path = path + [node]

            # ---------- 到达终点 ----------
            if node == end:
                total_time = cost
                # E_nodes = [n for n in path if ("E1" in n) or ("E2" in n)]
                E_nodes = []
                for i in range(len(path) - 1):  # 遍历到倒数第二个节点，避免越界
                    current_node = path[i]
                    next_node = path[i + 1]

                    # 检查当前节点和下一个节点是否都是电梯节点
                    if (
                            ("E1" in current_node) and ("E1" in next_node)
                    ) or (
                            ("E2" in current_node) and ("E2" in next_node)
                    ):
                        E_nodes.append(current_node)  # 加入当前节点
                        E_nodes.append(next_node)  # 加入下一个节点

                # # 去重（如果同一个节点可能被多次加入）
                # E_nodes = list(dict.fromkeys(E_nodes))  # 保持顺序去重

                # 如果少于两个 E 节点，返回安全默认值
                if len(E_nodes) < 2:
                    e1 = E_nodes[0] if len(E_nodes) >= 1 else None
                    e2 = None
                    return {
                        "path": path,
                        "total_time": total_time,
                        "segments": {"before": 0.0, "between": 0.0, "after": total_time},
                        "E_nodes": (e1, e2)  # 永远返回 tuple，不为 None
                    }

                # ---------- 正常情况 ----------
                e1 = E_nodes[0]
                e2 = E_nodes[1]
                idx_e1 = path.index(e1)
                idx_e2 = path.index(e2)

                before = 0.0
                between = 0.0
                after = 0.0

                for i in range(len(path) - 1):
                    u, v = path[i], path[i + 1]
                    w = get_edge_weight(u, v)
                    if i < idx_e1:
                        before += w
                    elif idx_e1 <= i < idx_e2:
                        between += w
                    else:
                        after += w

                return {
                    "path": path,
                    "total_time": total_time,
                    "segments": {"before": before, "between": between, "after": after},
                    "E_nodes": (e1, e2)
                }

            # ---------- 普通 Dijkstra 更新 ----------
            for neighbor, weight in self.edges.get(node, []):
                if neighbor not in visited:
                    heapq.heappush(pq, (cost + weight, neighbor, path))

        # ---------- 无法到达 ----------
        return {
            "path": [],
            "total_time": float("inf"),
            "segments": {"before": 0.0, "between": 0.0, "after": 0.0},
            "E_nodes": (None, None)
        }

    def dijkstra_extra_pro(self, start: str, end: str):
        """Enhanced Dijkstra: returns full path, total time, and safe segment times before/between/after E nodes."""
        import heapq

        def get_edge_weight(u, v):
            """Return weight of edge u->v, raise if not found."""
            for nb, w in self.edges.get(u, []):
                if nb == v:
                    return w
            raise ValueError(f"Edge weight not found for {u} -> {v}")

        pq = [(0, start, [])]
        visited = set()

        while pq:
            cost, node, path = heapq.heappop(pq)
            if node in visited:
                continue
            visited.add(node)
            path = path + [node]

            # ---------- 到达终点 ----------
            if node == end:
                total_time = cost
                E_nodes = [n for n in path if ("E1" in n) or ("E2" in n)]

                # 如果少于两个 E 节点，返回安全默认值
                if len(E_nodes) < 2 or len(E_nodes) == 3:
                    e1 = E_nodes[0] if len(E_nodes) >= 1 else None
                    e2 = None
                    return {
                        "path": path,
                        "total_time": total_time,
                        "segments": {"before": 0.0, "between_1": 0.0, "transfer": 0.0, "between": 0.0,
                                     "after": total_time},
                        "E_nodes": [(e1, e2)]  # 永远返回 tuple，不为 None
                    }
                elif len(E_nodes) == 2:

                    # ---------- 正常情况: 楼内移动 ----------
                    e1 = E_nodes[0]
                    e2 = E_nodes[1]
                    idx_e1 = path.index(e1)
                    idx_e2 = path.index(e2)

                    before = 0.0
                    between = 0.0
                    after = 0.0

                    for i in range(len(path) - 1):
                        u, v = path[i], path[i + 1]
                        w = get_edge_weight(u, v)
                        if i < idx_e1:
                            before += w
                        elif idx_e1 <= i < idx_e2:
                            between += w
                        else:
                            after += w

                    return {
                        "path": path,
                        "total_time": total_time,
                        "segments": {"before": before, "between_1": 0.0, "transfer": 0.0, "between": between,
                                     "after": after},
                        "E_nodes": [(e1, e2)]
                    }
                elif len(E_nodes) == 4:

                    # ---------- 正常情况: 楼间移动 ----------
                    e1 = E_nodes[0]
                    e2 = E_nodes[1]
                    e3 = E_nodes[2]
                    e4 = E_nodes[3]
                    idx_e1 = path.index(e1)
                    idx_e2 = path.index(e2)
                    idx_e3 = path.index(e3)
                    idx_e4 = path.index(e4)

                    before = 0.0
                    between_1 = 0.0
                    transfer = 0.0
                    between = 0.0
                    after = 0.0

                    for i in range(len(path) - 1):
                        u, v = path[i], path[i + 1]
                        w = get_edge_weight(u, v)
                        if i < idx_e1:
                            before += w
                        elif idx_e1 <= i < idx_e2:
                            between_1 += w
                        elif idx_e2 <= i < idx_e3:
                            transfer += w
                        elif idx_e3 <= i < idx_e4:
                            between += w
                        else:
                            after += w

                    return {
                        "path": path,
                        "total_time": total_time,
                        "segments": {"before": before, "between_1": between_1, "transfer": transfer, "between": between,
                                     "after": after},
                        "E_nodes": [(e1, e2), (e3, e4)]
                    }

            # ---------- 普通 Dijkstra 更新 ----------
            for neighbor, weight in self.edges.get(node, []):
                if neighbor not in visited:
                    heapq.heappush(pq, (cost + weight, neighbor, path))

        # ---------- 无法到达 ----------
        return {
            "path": [],
            "total_time": float("inf"),
            "segments": {"before": 0.0, "between_1": 0.0, "transfer": 0.0, "between": 0.0, "after": 0.0},
            "E_nodes": [(None, None)]
        }

    # def dijkstra(self, start: str, target: str):
    #     if start not in self.edges or target not in self.edges:
    #         return [], float("inf")
    #
    #     pq = [(0.0, start)]
    #     dist = {start: 0.0}
    #     prev: Dict[str, Optional[str]] = {start: None}
    #     visited = set()
    #
    #     while pq:
    #         cost, u = heapq.heappop(pq)
    #         if u in visited:
    #             continue
    #         visited.add(u)
    #
    #         if u == target:
    #             break
    #
    #         for v, w in self.edges.get(u, []):
    #             if v in visited:
    #                 continue
    #             nd = cost + w
    #             if nd < dist.get(v, float("inf")):
    #                 dist[v] = nd
    #                 prev[v] = u
    #                 heapq.heappush(pq, (nd, v))
    #
    #     if target not in dist:
    #         return [], float("inf")
    #
    #     # reconstruct path
    #     path = []
    #     cur: Optional[str] = target
    #     while cur is not None:
    #         path.append(cur)
    #         cur = prev.get(cur)
    #     path.reverse()
    #     return path, dist[target]
    #
    # def dijkstra_extra(self, start: str, target: str) -> Dict[str, Any]:
    #     # batch_scheduler 里主要用 path / total_time；其它字段给默认值保证兼容
    #     path, total = self.dijkstra(start, target)
    #     return {
    #         "path": path,
    #         "total_time": total,
    #         "segments": {
    #             "before": 0.0,
    #             "between_1": 0.0,
    #             "transfer": 0.0,
    #             "between": 0.0,
    #             "after": 0.0
    #         },
    #         "E_nodes": [(None, None)]
    #     }


def calculate_elevator_time_need(delta_floor: int) -> float:
    """
    电梯跨层耗时（与之前口径一致的默认公式）
    如果你后续要换成你们原 graph.py 的精确公式，只需改这一个函数。
    """
    delta_floor = abs(int(delta_floor))
    if delta_floor <= 0:
        return 0.0
    return 1.5 + 1.5 + 1.75 * delta_floor + 1.5
