
# src/core/yaml_graph_loader.py
import math
from typing import Dict, Any, Tuple, Optional
import yaml

from src.core.graph import Graph, calculate_elevator_time_need


def _parse_building_floor(node_name: str) -> Tuple[Optional[int], Optional[float]]:
    # 你的命名："{floor}_{building}_..."
    try:
        parts = node_name.split("_")
        if len(parts) >= 2:
            floor = float(parts[0])
            building = int(parts[1])
            return building, floor
    except Exception:
        pass
    return None, None


def _xy(node: Dict[str, Any]) -> Tuple[float, float]:
    c = node.get("node_coordinate", {}) or {}
    return float(c.get("x", 0.0)), float(c.get("y", 0.0))


def _is_elevator(node: Dict[str, Any]) -> bool:
    # 你最新口径：node_type=2 且 floor != -1000 为电梯
    try:
        t = int(node.get("node_type", 0))
        f = float(node.get("node_floor", 0))
        return (t == 2) and (abs(f - (-1000.0)) > 1e-9)
    except Exception:
        return False


def _group_elevators_by_xy(nodes_by_name: Dict[str, Dict[str, Any]], tol: float) -> Dict[Tuple[int, int], Dict[str, Any]]:
    """
    在每栋楼内按 (x,y) 聚类：
    - 距离 <= tol 认为同一部电梯
    返回 groups[(building, ex_idx)] = {"center":(x,y), "members":[node_names...]}
    ex_idx 从 1 开始，按 center 的 (x,y) 排序保证稳定
    """
    by_bld: Dict[int, list] = {}
    for name, nd in nodes_by_name.items():
        if not _is_elevator(nd):
            continue
        b, _ = _parse_building_floor(name)
        if b is None:
            continue
        by_bld.setdefault(b, []).append(name)

    groups: Dict[Tuple[int, int], Dict[str, Any]] = {}

    for b, names in by_bld.items():
        centers = []   # [(cx,cy)]
        buckets = []   # [[name,...]]

        for nm in names:
            x, y = _xy(nodes_by_name[nm])
            placed = False
            for i, (cx, cy) in enumerate(centers):
                if math.hypot(x - cx, y - cy) <= tol:
                    buckets[i].append(nm)
                    placed = True
                    break
            if not placed:
                centers.append((x, y))
                buckets.append([nm])

        # 稳定编号：按 center 排序
        order = sorted(range(len(centers)), key=lambda i: (centers[i][0], centers[i][1]))
        for ex_idx, i in enumerate(order, start=1):
            groups[(b, ex_idx)] = {"center": centers[i], "members": buckets[i]}

    return groups


def load_stair_graph_and_elevator_graphs(
    merged_nodes_yaml: str,
    elevator_xy_tol: float = 0.25,
    ensure_legacy_keys: bool = True,
    legacy_buildings=(1, 2, 3),
    legacy_elevators=(1, 2),
    speed_land=1.5,
    speed_stair=1.0
):
    """
    输入：merged_nodes.yaml（你现在 node_list/connect_info 格式）
    输出：
      stair_graph: Graph（直接按 connect_info 加边）
      elevator_graphs: dict[str, Graph]
        - 对真实出现的每部电梯："{building}_E{ex}" -> (stair_graph + 该电梯跨层边)
        - 可选：补齐 legacy keys（缺的映射为 stair_graph，避免 batch_scheduler KeyError）
    """
    with open(merged_nodes_yaml, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    node_list = data.get("node_list", None)
    if node_list is None:
        # 兼容你可能写成 ode_list（你贴出来就是这个 typo）
        node_list = data.get("ode_list", None)
    if not isinstance(node_list, list):
        raise ValueError("merged_nodes.yaml 顶层必须包含 node_list(或 ode_list) 且为 list")

    nodes_by_name: Dict[str, Dict[str, Any]] = {}
    for nd in node_list:
        nm = nd.get("node_name")
        if nm:
            nodes_by_name[str(nm)] = nd

    # 1) stair_graph：直接用 connect_info 的 weight（已经是时间）
    stair_graph = Graph()
    for nm in nodes_by_name.keys():
        stair_graph.add_node(nm)

    added = set()
    for u, nd in nodes_by_name.items():
        for c in (nd.get("connect_info") or []):
            v = c.get("to")
            if not v or v not in nodes_by_name:
                continue
            key = (u, v) if u < v else (v, u)
            if key in added:
                continue
            added.add(key)

            w = float(c.get("weight", 0.0))
            bi = bool(c.get("bidirectional", True))
            stair_graph.add_edge(u, v, w, bidirectional=bi)

    # 2) 电梯分组（按 building 内 (x,y) 聚类） => E1/E2/...
    groups = _group_elevators_by_xy(nodes_by_name, tol=elevator_xy_tol)

    # 3) 为每部电梯构建电梯图：复制 stair_graph 边，再加该电梯的跨层边
    def clone_graph(g: Graph) -> Graph:
        ng = Graph()
        for u, lst in g.edges.items():
            ng.edges[u] = list(lst)
        return ng

    elevator_graphs: Dict[str, Graph] = {}

    for (b, ex), info in groups.items():
        g = clone_graph(stair_graph)

        # 该电梯在不同楼层的节点名：按 node_floor 分组
        floor_to_node: Dict[int, str] = {}
        for nm in info["members"]:
            fl = nodes_by_name[nm].get("node_floor", None)
            if fl is None:
                continue
            try:
                fl_i = int(round(float(fl)))
            except Exception:
                continue
            floor_to_node[fl_i] = nm

        floors = sorted(floor_to_node.keys())
        # 按你要求“电梯可到每一层”：全连（任意两层互连）
        for i in range(len(floors)):
            for j in range(i + 1, len(floors)):
                f1, f2 = floors[i], floors[j]
                n1, n2 = floor_to_node[f1], floor_to_node[f2]
                w = calculate_elevator_time_need(abs(f2 - f1))
                g.add_edge(n1, n2, w, bidirectional=True)

        elevator_graphs[f"{b}_E{ex}"] = g

    # 4) 兼容你当前 batch_scheduler 硬编码 keys：补齐缺失 keys 为 stair_graph（不崩）
    if ensure_legacy_keys:
        for b in legacy_buildings:
            for ex in legacy_elevators:
                k = f"{b}_E{ex}"
                elevator_graphs.setdefault(k, stair_graph)

    return stair_graph, elevator_graphs
