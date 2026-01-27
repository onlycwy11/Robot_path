#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
template_to_topo_fused.py (完整最终版：不移动电梯坐标)
======================================================
只从 data/ 目录读取一批 *_topo.yaml（含楼层文件 + 单独的 stairs 文件），输出一个 merged_nodes.yaml。

【输入语义（你最新要求）】
- node_type = 1 或 3：房间（room）
- node_type = 2 且 floor != -1000 且 不在 stairs_*.yaml 且 楼层为整数层：电梯（elevator）
- stairs_*.yaml：楼梯层文件（连接上下两层）
  * 通过 stairs 文件内节点的 connect[].floor 推断 midfloor（如 4.5），并把该 stairs 文件里所有节点的 floor 置为 midfloor
  * stairs 文件里的点不当电梯（即使 node_type=2）

【电梯区分规则（你最新要求）】
- 同层若两个电梯点的 (x,y) 不同 => 两个不同电梯（Ex 不同）
- (x,y) 相同 => 同一个电梯（Ex 相同）
- 不对电梯位置作任何移动/偏移
- 为了兼容浮点误差，可用 tol_m（默认 0.0 = 严格相同；可改成 0.001/0.01）

【电梯补齐 + 同层连接复制（你要求的精确定义）】
- 对每栋楼的每部电梯（由 (x,y) 分组得到 Ex）：
  1) 在该栋楼所有“非楼梯层”（整数楼层）上，都补齐该电梯节点，且 (x,y) 使用该电梯组的原始 (x,y)
  2) 若某层原本就有电梯节点：新建/标准化电梯节点连接“该层原本电梯连接的房间点”
     - 只考虑房间点（node_type=1/3，对应名字 ..._pY），不考虑 connect 跨图
     - 我们从该层电梯节点的 neighbors/已有 conn_map 里提取其连接的房间点集合
  3) 若某层电梯是补建的（或该层电梯没有房间连接）：用“参考层电梯”的房间 local-id(pY)集合在该层找同名房间连接

【边权重（graph.py 语义：时间代价）】
- 平地：weight = dist(m) / 1.5
- 楼梯：weight = dist(m) / 0.5   （只要边的任一端在半层 midfloor，即视作楼梯边）
- 电梯跨层：weight = calculate_elevator_time_need(|Δfloor|)，
    calculate_elevator_time_need(n) = 1.5 + 1.5 + 1.75*n + 1.5

【输出 merged_nodes.yaml】
每个节点仅 5 字段：
- node_name
- node_coordinate {x,y,z,yaw,unit}
- node_floor (float 允许 4.5)
- node_type (保持原始 1/2/3)
- connect_info: [{to, mode, bidirectional, weight}]

template_to_topo.py (最终版：稳健输出合法 YAML + 原子写入)
======================================================
只从 data/ 目录读取一批 *_topo.yaml（含楼层文件 + 单独的 stairs 文件），输出一个 merged_nodes.yaml。

重点修复：
1) building 不再硬编码为 1，而是使用 building_map 或从 map_name 推断
2) 输出采用“原子写入”：先写临时文件，再 os.replace，避免中途崩溃留下半截 YAML
3) 输出前做节点字段健全性检查（x/y/z/yaw/unit 必须存在且可转 float）
4) 入口函数恢复为 main()，并启用 if __name__ == "__main__"
"""

import os
import re
import sys
import glob
import math
import argparse
from collections import defaultdict
from typing import Any, Dict, List, Tuple, Optional

import yaml


# ----------------------------
# graph.py elevator time model
# ----------------------------
def calculate_elevator_time_need(n: int) -> float:
    return 1.5 + 1.5 + 1.75 * n + 1.5


SPEED_LAND = 1.5   # m/s
SPEED_STAIR = 0.5  # m/s


# ----------------------------
# helpers
# ----------------------------
def infer_map_name(filepath: str) -> str:
    base = os.path.basename(filepath)
    for suffix in ("_topo.yaml", "_topo.yml"):
        if base.endswith(suffix):
            return base[: -len(suffix)]
    return os.path.splitext(base)[0]


def parse_neighbors(raw: Any) -> List[str]:
    if raw is None:
        return []
    # 有些 topo 里 param[4] 可能是 list/tuple，也兼容一下
    if isinstance(raw, (list, tuple)):
        out: List[str] = []
        for x in raw:
            if x is None:
                continue
            out.extend(str(x).split(","))
        return [s.strip() for s in out if s.strip()]
    s = str(raw).strip()
    if not s:
        return []
    return [t.strip() for t in s.split(",") if t.strip()]


def safe_float(x: Any, default: float = 0.0) -> float:
    try:
        return float(x)
    except Exception:
        return default


def safe_int(x: Any, default: int = 0) -> int:
    try:
        return int(x)
    except Exception:
        return default


def is_integer_floor(f: float) -> bool:
    return abs(f - round(f)) < 1e-9


def fmt_floor(f: float) -> str:
    if is_integer_floor(f):
        return str(int(round(f)))
    s = str(f)
    return s.rstrip("0").rstrip(".")


def dist_m(x1: float, y1: float, x2: float, y2: float) -> float:
    return math.hypot(x1 - x2, y1 - y2)


def is_stairs_map(map_name: str) -> bool:
    return map_name.lower().startswith("stairs")


def load_building_map(path: str) -> Dict[str, int]:
    if not path:
        return {}
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    m = data.get("map_to_building") or {}
    out = {}
    for k, v in m.items():
        try:
            out[str(k)] = int(v)
        except Exception:
            continue
    return out


def infer_building_id(map_name: str, map_to_building: Dict[str, int]) -> int:
    # if map_name in map_to_building:
    #     return map_to_building[map_name]
    # nums = re.findall(r"(\d+)", map_name)
    # if nums:
    #     return int(nums[-1])
    return 1


# ----------------------------
# stairs midfloor inference from stairs yaml connect floors
# ----------------------------
def infer_stairs_midfloor_for_map(map_name: str, orig_nodes: Dict[str, Dict[str, Any]]) -> Optional[float]:
    """
    stairs 文件中，通过 connect[].floor 推断它连接的上下两层 (例如 4 和 5)，得到 midfloor=4.5。
    - 若只看到一侧 floor=4，则默认 midfloor=4.5。
    - 若看到非相邻（例如 3 和 5），返回 None（不强行推断）。
    """
    floors: List[int] = []
    for _old, rec in orig_nodes.items():
        if rec["map_name"] != map_name:
            continue
        for c in (rec.get("connect") or []):
            f = c.get("floor", None)
            if f is None:
                continue
            try:
                fi = int(f)
            except Exception:
                continue
            floors.append(fi)

    floors = sorted(set(floors))
    if len(floors) >= 2:
        lo, hi = floors[0], floors[-1]
        if hi - lo == 1:
            return lo + 0.5
        return None
    if len(floors) == 1:
        return floors[0] + 0.5
    return None


# ----------------------------
# elevator candidate
# ----------------------------
def is_elevator_candidate(rec: Dict[str, Any]) -> bool:
    """
    node_type=2 且 floor!=-1000 且 不在 stairs 文件 且 楼层整数层 => 电梯候选
    """
    if int(rec["node_type"]) != 2:
        return False
    if is_stairs_map(rec["map_name"]):
        return False

    # ✅ 新增：如果它 connect 到 stairs_*，说明它是“楼梯口/跨图连接点”，不要当电梯
    for c in (rec.get("connect") or []):
        tgt_map = str(c.get("map_name", "")).strip()
        if tgt_map and is_stairs_map(tgt_map):
            return False

    f = float(rec["node_floor"])
    if abs(f - (-1000.0)) < 1e-9:
        return False
    if not is_integer_floor(f):
        return False
    return True


# ----------------------------
# group elevators by (x,y) WITHOUT moving coordinates
# ----------------------------
def group_elevators_by_xy_hash(
    elev_points: List[Tuple[str, int, int, float, float]],
    tol_m: float = 0.0
) -> Tuple[Dict[str, int], Dict[Tuple[int, int], Tuple[float, float]]]:
    """
    以 (x,y) 是否相同来区分电梯，不做任何移动/偏移。

    elev_points: (old_node_key, building, floor_int, x, y)

    tol_m:
      - 0.0 表示严格相同
      - >0 表示在 tol 网格内认为相同（用于浮点误差）

    Returns:
      old_to_eidx: old_node_key -> Ex (1..K per building)
      group_xy: (building, Ex) -> (x,y)（为该组的代表坐标：tol=0 即原坐标；tol>0 为量化坐标）
    """
    by_building = defaultdict(list)
    for old, b, fl, x, y in elev_points:
        by_building[b].append((old, x, y))

    def key_xy(x: float, y: float) -> Tuple[float, float]:
        if tol_m <= 0.0:
            return (x, y)
        qx = round(x / tol_m) * tol_m
        qy = round(y / tol_m) * tol_m
        return (qx, qy)

    old_to_eidx: Dict[str, int] = {}
    group_xy: Dict[Tuple[int, int], Tuple[float, float]] = {}

    for b, items in by_building.items():
        uniq = sorted({key_xy(x, y) for (_old, x, y) in items}, key=lambda t: (t[0], t[1]))
        xy_to_idx = {xy: i + 1 for i, xy in enumerate(uniq)}
        for idx, xy in enumerate(uniq, start=1):
            group_xy[(b, idx)] = (xy[0], xy[1])
        for old, x, y in items:
            old_to_eidx[old] = xy_to_idx[key_xy(x, y)]

    return old_to_eidx, group_xy


# ----------------------------
# naming helpers
# ----------------------------
_elev_name_pat = re.compile(r"^([0-9]+(?:\.[0-9]+)?)_([0-9]+)_E([0-9]+)$")
_room_name_pat = re.compile(r"^([0-9]+(?:\.[0-9]+)?)_([0-9]+)_(p\d+)$", re.IGNORECASE)


def parse_elevator_name(name: str):
    m = _elev_name_pat.match(name)
    if not m:
        return None
    return float(m.group(1)), int(m.group(2)), int(m.group(3))


def room_local_id(name: str) -> Optional[str]:
    """
    仅把形如 {floor}_{building}_pY 的点当“房间点”，local_id = pY
    """
    m = _room_name_pat.match(name)
    if not m:
        return None
    return m.group(3)


# ----------------------------
# connect_info helpers
# ----------------------------
def add_conn(conn_map: dict, u: str, v: str, mode: str, w: float, bidirectional: bool = True):
    conn_map.setdefault(u, [])
    keyset = {(c["to"], c["mode"], bool(c["bidirectional"]), round(float(c["weight"]), 6)) for c in conn_map[u]}
    k = (v, mode, bool(bidirectional), round(float(w), 6))
    if k not in keyset:
        conn_map[u].append({"to": v, "mode": mode, "bidirectional": bool(bidirectional), "weight": float(w)})


def edge_mode_and_weight(merged_nodes: dict, u: str, v: str) -> Tuple[str, float]:
    """
    mode:
      - elevator: elevator<->elevator 且楼层不同
      - stair: 任一端点为半层(midfloor) => stair
      - walk: 其他
    """
    fu = float(merged_nodes[u]["node_floor"])
    fv = float(merged_nodes[v]["node_floor"])

    eu = parse_elevator_name(u) is not None
    ev = parse_elevator_name(v) is not None

    if eu and ev and abs(fu - fv) > 1e-9:
        n = int(round(abs(fu - fv)))
        return "elevator", float(calculate_elevator_time_need(n))

    cu = merged_nodes[u]["node_coordinate"]
    cv = merged_nodes[v]["node_coordinate"]
    d = dist_m(float(cu["x"]), float(cu["y"]), float(cv["x"]), float(cv["y"]))

    if (not is_integer_floor(fu)) or (not is_integer_floor(fv)):
        return "stair", float(d / SPEED_STAIR)

    return "walk", float(d / SPEED_LAND)


# ----------------------------
# elevator shafts + per-floor room connections + vertical elevator edges
# ----------------------------
def build_elevator_shafts_and_edges(
    merged_nodes: dict,
    conn_map: dict,
    group_xy: Dict[Tuple[int, int], Tuple[float, float]],
):
    """

    - 每栋楼每个电梯(Ex)在所有整数楼层补齐节点，且 (x,y) 为该组的原始 (x,y)（不移动）
    - 若某层原本有电梯节点：标准化电梯节点连接该层原本电梯连接的房间
    - 若补建层无连接：复制参考层电梯连接的 pY
    - 同一 Ex 不同层之间加 elevator 边（全连）
    """

    # 1) 每栋楼有哪些整数楼层
    floors_by_building = defaultdict(set)
    rooms_by_floor_building_local = defaultdict(dict)  # (floor_int, building) -> {pY: node_name}
    z_by_floor_building = defaultdict(list)

    for name, rec in merged_nodes.items():
        parts = name.split("_")
        if len(parts) < 2:
            continue
        try:
            building = int(parts[1])
        except Exception:
            continue

        f = float(rec["node_floor"])
        if is_integer_floor(f):
            fi = int(round(f))
            floors_by_building[building].add(fi)
            z_by_floor_building[(fi, building)].append(float(rec["node_coordinate"].get("z", 0.0)))

        rid = room_local_id(name)
        if rid and is_integer_floor(f):
            rooms_by_floor_building_local[(int(round(f)), building)][rid] = name

    # 2) 当前已存在的电梯节点（按 building,ex,floor）
    elev_by_floor: Dict[Tuple[int, int, int], str] = {}
    for name in list(merged_nodes.keys()):
        pe = parse_elevator_name(name)
        if not pe:
            continue
        f, b, ex = pe
        if not is_integer_floor(f):
            continue
        elev_by_floor[(b, ex, int(round(f)))] = name

    # helper：从电梯节点当前 conn_map 里提取它连接到的房间 local-ids（pY）
    def elevator_room_local_ids(elev_name: str) -> set:
        out = set()
        for c in conn_map.get(elev_name, []):
            rid = room_local_id(str(c.get("to", "")))
            if rid:
                out.add(rid)
        return out

    # 3) 对每栋楼每个 Ex：逐层补齐 & 连接复制
    for (b, ex), (base_x, base_y) in sorted(group_xy.items()):
        target_floors = sorted(floors_by_building.get(b, set()))
        if not target_floors:
            continue

        # 3.1 找参考连接模板：选择“已有电梯层中，房间连接最多的那层”
        ref_room_ids: set = set()
        best_cnt = -1
        for fl in target_floors:
            nm = elev_by_floor.get((b, ex, fl))
            if not nm:
                continue
            ids = elevator_room_local_ids(nm)
            if len(ids) > best_cnt:
                best_cnt = len(ids)
                ref_room_ids = ids

        # 3.2 逐层
        for fl in target_floors:
            canonical_name = f"{fl}_{b}_E{ex}"
            nm = elev_by_floor.get((b, ex, fl), canonical_name)
            elev_by_floor[(b, ex, fl)] = nm

            # 创建或更新电梯节点（不移动：直接设为 group_xy 的 base_x/base_y）
            if nm not in merged_nodes:
                zs = z_by_floor_building.get((fl, b), [])
                z = sum(zs) / len(zs) if zs else 0.0
                merged_nodes[nm] = {
                    "node_name": nm,
                    "node_coordinate": {"x": float(base_x), "y": float(base_y), "z": float(z), "yaw": 0.0, "unit": "m"},
                    "node_floor": float(fl),
                    "node_type": 2,
                }
                conn_map.setdefault(nm, [])
            else:
                merged_nodes[nm]["node_coordinate"]["x"] = float(base_x)
                merged_nodes[nm]["node_coordinate"]["y"] = float(base_y)

            # 该层应连接的房间集合：
            # - 若该层原本电梯已有房间连接：用它
            # - 否则用参考层 ref_room_ids
            current_ids = elevator_room_local_ids(nm)
            target_room_ids = current_ids if current_ids else ref_room_ids
            if not target_room_ids:
                continue

            room_map = rooms_by_floor_building_local.get((fl, b), {})
            if not room_map:
                continue

            cu = merged_nodes[nm]["node_coordinate"]
            ux, uy = float(cu["x"]), float(cu["y"])

            for rid in sorted(target_room_ids):
                if rid not in room_map:
                    continue
                v = room_map[rid]
                cv = merged_nodes[v]["node_coordinate"]
                vx, vy = float(cv["x"]), float(cv["y"])
                d = dist_m(ux, uy, vx, vy)
                w = d / SPEED_LAND
                add_conn(conn_map, nm, v, "walk", w, bidirectional=True)
                add_conn(conn_map, v, nm, "walk", w, bidirectional=True)

    # 4) 跨层电梯边（全连）
    by_elev = defaultdict(list)  # (b,ex) -> list[(floor, name)]
    for (b, ex, fl), nm in elev_by_floor.items():
        by_elev[(b, ex)].append((fl, nm))

    for (b, ex), items in by_elev.items():
        items = sorted(items, key=lambda t: t[0])
        n = len(items)
        for i in range(n):
            fi, ni = items[i]
            for j in range(i + 1, n):
                fj, nj = items[j]
                diff = abs(fj - fi)
                w = float(calculate_elevator_time_need(diff))
                add_conn(conn_map, ni, nj, "elevator", w, bidirectional=True)
                add_conn(conn_map, nj, ni, "elevator", w, bidirectional=True)


# ----------------------------
# output safety
# ----------------------------
def _validate_merged_nodes_for_yaml(merged_nodes: Dict[str, Dict[str, Any]]):
    """
    防止出现你之前那种 y: node_name 粘行/缺字段：我们强制保证每个 node_coordinate 有完整字段且可 float。
    """
    for name, rec in merged_nodes.items():
        if "node_coordinate" not in rec or not isinstance(rec["node_coordinate"], dict):
            raise ValueError(f"[BAD NODE] {name}: missing node_coordinate dict")
        c = rec["node_coordinate"]
        # 缺字段就补默认
        c.setdefault("x", 0.0)
        c.setdefault("y", 0.0)
        c.setdefault("z", 0.0)
        c.setdefault("yaw", 0.0)
        c.setdefault("unit", "m")
        # 强制可转 float（避免 None / 字符串异常）
        try:
            c["x"] = float(c["x"])
            c["y"] = float(c["y"])
            c["z"] = float(c["z"])
            c["yaw"] = float(c["yaw"])
        except Exception as e:
            raise ValueError(f"[BAD NODE] {name}: invalid coordinate values: {c}, err={e}")


def _atomic_dump_yaml(obj: Dict[str, Any], out_path: str):
    """
    原子写入：先写 out_path.tmp，再 replace 到 out_path
    避免程序中断导致 out_path 留下半截 YAML（你之前的坏文件很像这种情况）
    """
    out_dir = os.path.dirname(os.path.abspath(out_path))
    os.makedirs(out_dir, exist_ok=True)

    tmp_path = out_path + ".tmp"
    with open(tmp_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(
            obj,
            f,
            allow_unicode=True,
            sort_keys=False,
            default_flow_style=False,
            width=120,
            indent=2,
        )
    os.replace(tmp_path, out_path)


# ----------------------------
# main
# ----------------------------
def building_graph(campus_name: str = "new_campus"):
    ap = argparse.ArgumentParser()
    default_dir = os.path.dirname(os.path.abspath(__file__))

    # ap.add_argument("--external-dir", default=os.path.join(default_dir, "data"),
    #                 help="包含 *_topo.yaml 的目录（默认：src/core/data）")
    # ap.add_argument("--out", default=os.path.join(default_dir, "data", campus_name, "merged_nodes.yaml"),
    #                 help="输出 merged_nodes.yaml（默认：src/core/data/new_campus/merged_nodes.yaml）")
    # ap.add_argument("--building-map", default="",
    #                 help="可选：building_map.yaml，格式 map_to_building: {map_name: building_id}")
    # ap.add_argument("--elevator-tol", type=float, default=0.0,
    #                 help="电梯 (x,y) 判等容差(米)。0=严格相同；推荐 0.001(1mm) 或 0.01(1cm) 处理浮点误差。")
    external_dir = os.path.join(default_dir, "data", campus_name, "yaml")  # 默认数据目录
    out = os.path.join(default_dir, "data", campus_name, "merged_nodes.yaml")  # 输出文件路径
    building_map = ""  # 默认空字符串（可选参数）
    elevator_tol = 0.00  # 默认容差（1mm）

    # 1. 定义参数（但不再用 default=...，而是手动计算默认值）
    ap.add_argument("--external-dir")  # 不指定 default，后面手动赋值
    ap.add_argument("--out")
    ap.add_argument("--building-map")
    ap.add_argument("--elevator-tol", type=float)

    print(campus_name)

    # 2. 手动构造 "伪命令行参数列表"，基于 campus_name 设置默认值
    args_list = [
        "--external-dir", external_dir,  # 手动设置默认值
        "--out", out,  # 基于 campus_name
        "--building-map", building_map,  # 默认空字符串
        "--elevator-tol", elevator_tol  # 默认容差 1mm
    ]

    # 3. 解析 "伪参数列表"，而不是真的从命令行读取
    args, _ = ap.parse_known_args(args_list)  # 解析我们构造的参数列表

    map_to_building = load_building_map(args.building_map) if args.building_map else {}

    files = sorted(set(
        glob.glob(os.path.join(args.external_dir, "*_topo.yaml")) +
        glob.glob(os.path.join(args.external_dir, "*_topo.yml"))
    ))
    if not files:
        raise FileNotFoundError(f"No *_topo.yaml found in: {args.external_dir}")

    # 1) 读取所有节点
    orig_nodes: Dict[str, Dict[str, Any]] = {}
    orig_edges_raw: List[Tuple[str, str, bool]] = []  # (old_u, old_v, bidirectional)

    for fp in files:
        map_name = infer_map_name(fp)
        building = infer_building_id(map_name, map_to_building)
        # building = 1

        with open(fp, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}

        for n in (data.get("node_list") or []):
            local = str(n.get("node_name", "")).strip()
            if not local:
                continue
            old = f"{map_name}:{local}"

            floor = safe_float(n.get("floor", 0), 0.0)
            ntype = safe_int(n.get("node_type", 0), 0)

            param = n.get("param", []) or []
            if len(param) < 5:
                param = list(param) + [0.0] * (5 - len(param))
            x, y, z, yaw = float(param[0]), float(param[1]), float(param[2]), float(param[3])
            neigh = parse_neighbors(param[4])

            orig_nodes[old] = {
                "map_name": map_name,
                "local_name": local,
                "building": building,
                "node_floor": floor,
                "node_type": ntype,
                "coord": {"x": x, "y": y, "z": z, "yaw": yaw, "unit": "m"},
                "neighbors": neigh,
                "connect": n.get("connect") or [],
            }

    # 2) 推断 stairs midfloor，并覆盖 stairs 文件内所有节点的 floor
    stairs_midfloor: Dict[str, float] = {}
    for fp in files:
        mn = infer_map_name(fp)
        if not is_stairs_map(mn):
            continue
        mid = infer_stairs_midfloor_for_map(mn, orig_nodes)
        if mid is not None:
            stairs_midfloor[mn] = float(mid)

    for _old, rec in orig_nodes.items():
        mn = rec["map_name"]
        if is_stairs_map(mn) and mn in stairs_midfloor:
            rec["node_floor"] = float(stairs_midfloor[mn])

    # 3) raw edges：neighbors 全加；connect 只用于非电梯相关（你说电梯不考虑 connect）
    for old_u, rec in orig_nodes.items():
        map_name = rec["map_name"]

        # neighbors (同图)
        for nb_local in rec["neighbors"]:
            old_v = f"{map_name}:{nb_local}"
            if old_v in orig_nodes:
                orig_edges_raw.append((old_u, old_v, True))

        # connect (跨图)：若任一端为电梯候选，则跳过
        for c in (rec["connect"] or []):
            tgt_map = str(c.get("map_name", "")).strip()
            tgt_local = str(c.get("target_node", "")).strip()
            if not tgt_map or not tgt_local:
                continue
            old_v = f"{tgt_map}:{tgt_local}"
            if old_v not in orig_nodes:
                continue
            if is_elevator_candidate(orig_nodes[old_u]) or is_elevator_candidate(orig_nodes[old_v]):
                continue
            orig_edges_raw.append((old_u, old_v, True))

    # 4) 电梯按 (x,y) 分组得到 Ex（不移动坐标）
    elev_points: List[Tuple[str, int, int, float, float]] = []
    for old, rec in orig_nodes.items():
        if not is_elevator_candidate(rec):
            continue
        b = int(rec["building"])
        fl = int(round(float(rec["node_floor"])))
        c = rec["coord"]
        elev_points.append((old, b, fl, float(c["x"]), float(c["y"])))

    old_to_eidx, group_xy = group_elevators_by_xy_hash(elev_points, tol_m=float(args.elevator_tol))
    print("[DEBUG] Elevator groups (building, Ex) -> (x,y):")
    for (b, ex), (x, y) in sorted(group_xy.items()):
        print(f"  building={b} E{ex}: x={x} y={y}")
    print()


    # 5) old->new 命名
    old_to_new: Dict[str, str] = {}
    for old, rec in orig_nodes.items():
        b = int(rec["building"])
        f = float(rec["node_floor"])
        local = rec["local_name"]

        if is_elevator_candidate(rec):
            ex = old_to_eidx.get(old, None)
            if ex is None:
                # 极少 fallback：为该楼新增一个 Ex（仍不移动坐标）
                max_ex = max([ee for (bb, ee) in group_xy.keys() if bb == b], default=0)
                ex = max_ex + 1
                c = rec["coord"]
                group_xy[(b, ex)] = (float(c["x"]), float(c["y"]))
            new_name = f"{fmt_floor(f)}_{b}_E{ex}"
        else:
            new_name = f"{fmt_floor(f)}_{b}_{local}"

        old_to_new[old] = new_name

    # 6) 合并同名节点（平均坐标）
    acc: Dict[str, Dict[str, Any]] = {}
    for old, rec in orig_nodes.items():
        new = old_to_new[old]
        c = rec["coord"]
        if new not in acc:
            acc[new] = {
                "sum_x": 0.0, "sum_y": 0.0, "sum_z": 0.0, "sum_yaw": 0.0, "cnt": 0,
                "node_floor": float(rec["node_floor"]),
                "node_type": int(rec["node_type"]),
            }
        a = acc[new]
        a["sum_x"] += float(c["x"])
        a["sum_y"] += float(c["y"])
        a["sum_z"] += float(c["z"])
        a["sum_yaw"] += float(c.get("yaw", 0.0))
        a["cnt"] += 1

    merged_nodes: Dict[str, Dict[str, Any]] = {}
    for name, a in acc.items():
        cnt = max(1, int(a["cnt"]))
        merged_nodes[name] = {
            "node_name": name,
            "node_coordinate": {
                "x": a["sum_x"] / cnt,
                "y": a["sum_y"] / cnt,
                "z": a["sum_z"] / cnt,
                "yaw": a["sum_yaw"] / cnt,
                "unit": "m",
            },
            "node_floor": float(a["node_floor"]),
            "node_type": int(a["node_type"]),
        }

    # 7) raw edges -> conn_map（含 weight）
    conn_map: Dict[str, List[Dict[str, Any]]] = defaultdict(list)
    seen = set()

    for old_u, old_v, bi in orig_edges_raw:
        u = old_to_new.get(old_u)
        v = old_to_new.get(old_v)
        if not u or not v:
            continue
        if u == v:
            continue
        if u not in merged_nodes or v not in merged_nodes:
            continue

        mode, w = edge_mode_and_weight(merged_nodes, u, v)
        key = (u, v, mode, bool(bi), round(float(w), 6))
        if key in seen:
            continue
        seen.add(key)

        add_conn(conn_map, u, v, mode, w, bidirectional=bool(bi))
        if bi:
            add_conn(conn_map, v, u, mode, w, bidirectional=bool(bi))

    # 8) 电梯补齐 + 连接复制 + 跨层电梯边
    build_elevator_shafts_and_edges(merged_nodes, conn_map, group_xy)

    # 9) conn_map 去重
    for u in list(conn_map.keys()):
        out = []
        s2 = set()
        for c in conn_map[u]:
            k = (c["to"], c["mode"], bool(c["bidirectional"]), round(float(c["weight"]), 6))
            if k in s2:
                continue
            s2.add(k)
            out.append(c)
        conn_map[u] = out

    # ✅ 关键：输出前做健全性检查，防止缺 y / None / 乱格式
    _validate_merged_nodes_for_yaml(merged_nodes)

    # 10) 导出 merged_nodes.yaml（仅 5 字段）
    node_list = []
    for name in sorted(merged_nodes.keys()):
        rec = merged_nodes[name]
        node_list.append({
            "node_name": rec["node_name"],
            "node_coordinate": rec["node_coordinate"],
            "node_floor": float(rec["node_floor"]),
            "node_type": int(rec["node_type"]),
            "connect_info": conn_map.get(name, []),
        })

    out_obj = {"node_list": node_list}

    # ✅ 原子写入：避免中断写坏 YAML
    _atomic_dump_yaml(out_obj, args.out)
    # os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    # with open(args.out, "w", encoding="utf-8") as f:
    #     yaml.safe_dump(out_obj, f, allow_unicode=True, sort_keys=False, width=120)

    print(f"Wrote: {args.out}")
    print(f"Input topo files: {len(files)}")
    print(f"Original nodes: {len(orig_nodes)} -> Merged nodes: {len(merged_nodes)}")
    if stairs_midfloor:
        print("Stairs maps inferred midfloors:")
        for k in sorted(stairs_midfloor.keys()):
            print(f"  {k}: {stairs_midfloor[k]}")
    print(f"Elevators grouped: {len(group_xy)}  (tol={args.elevator_tol}m)")
    print(f"Speeds: land={SPEED_LAND} m/s, stair={SPEED_STAIR} m/s")


# if __name__ == "__main__":
#     building_graph("sandun")
