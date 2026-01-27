"""
路径选择模块

提供最优路径选择功能，包括楼梯路径和电梯路径的比较。
"""

import math
import copy
from typing import Dict, List, Tuple, Optional, Any

from src.core.graph import Graph
from src.models.elevator import Elevator
from src.models.task import Task
from src.models.path_info import PathInfo, Assignment
from src.utils.config import DEFAULT_CONFIG, SystemConfig
from src.utils.position_utils import (
    parse_position,
    get_building_from_node,
    needs_two_elevators,
    create_elevator_access_node,
)
from src.utils.path_utils import merge_paths
from src.utils.constants import PathPhase, PATH_PHASE_KEYS


def select_best_path_with_elevator(
    task: Task,
    rid: int,
    start_pos: str,
    pickup_pos: str,
    target_pos: str,
    stair_graph: Graph,
    graph_map: Dict[str, Graph],
    elevators: Dict[str, Elevator],
    current_time: float,
    available_time: float,
    campus_name: str,
    config: SystemConfig = DEFAULT_CONFIG
) -> Tuple[Optional[PathInfo], Optional[PathInfo], Dict[str, PathInfo]]:
    """
    选择最优路径（比较楼梯和电梯路径）

    Args:
        task: 任务对象
        rid: 机器人 ID
        start_pos: 机器人起始位置
        pickup_pos: 取货位置（药品位置）
        target_pos: 送货位置（目标位置）
        stair_graph: 楼梯图
        graph_map: 电梯增强图字典
        elevators: 电梯字典
        current_time: 当前时间
        available_time: 机器人可用时间
        campus_name: 校园名称
        config: 系统配置

    Returns:
        (取货最优路径信息, 送货最优路径信息, 所有路径结果字典)
    """
    path_results: Dict[str, PathInfo] = {}
    tid = task.id

    # 楼梯路径计算
    if pickup_pos:
        # 取货阶段
        path_stair_pickup, cost_stair_pickup = stair_graph.dijkstra(start_pos, pickup_pos)
        if path_stair_pickup and not math.isinf(cost_stair_pickup):
            path_results["stair_1"] = {
                "path": path_stair_pickup,
                "start_time": available_time,
                "actual_time": cost_stair_pickup,
                "part_time_2": cost_stair_pickup,
                "wait_time_2": 0.0,
                "before": 0.0,
                "between": 0.0,
                "after": 0.0,
                "eid_2": None,
                "type": "stair",
                "status": PathPhase.PICK.value,
            }

        # 送货阶段
        path_stair_delivery, cost_stair_delivery = stair_graph.dijkstra(pickup_pos, target_pos)
    else:
        path_stair_delivery, cost_stair_delivery = stair_graph.dijkstra(start_pos, target_pos)

    if path_stair_delivery and not math.isinf(cost_stair_delivery):
        path_results["stair_2"] = {
            "path": path_stair_delivery,
            "start_time": available_time,
            "actual_time": cost_stair_delivery,
            "part_time_2": cost_stair_delivery,
            "wait_time_2": 0.0,
            "before": 0.0,
            "between": 0.0,
            "after": 0.0,
            "eid": None,
            "type": "stair",
            "status": PathPhase.DELIVER.value
        }

    # 电梯路径计算
    start = start_pos
    if pickup_pos:
        segment_index = PathPhase.PICK.value
        target = pickup_pos
    else:
        segment_index = PathPhase.DELIVER.value
        target = target_pos

    actual_time = 0.0

    while segment_index <= PathPhase.DELIVER.value:
        for eid, g in graph_map.items():
            # 跨楼运输（需要两部电梯）
            if needs_two_elevators(start, target):
                transit = create_elevator_access_node(start, eid)
                building_num = get_building_from_node(start)

                if eid.split("_")[0] == building_num:
                    res_1 = g.dijkstra_extra(start, transit)
                    if not res_1 or "total_time" not in res_1 or not res_1["path"]:
                        continue

                    for eid_1, g_1 in graph_map.items():
                        target_building = get_building_from_node(target)
                        if eid_1.split("_")[0] == target_building:
                            res_2 = g_1.dijkstra_extra(transit, target)
                            if not res_2 or "total_time" not in res_2 or not res_2["path"]:
                                continue

                            # 提取分段信息
                            path_info = _build_two_elevator_path_info(
                                res_1, res_2, eid, eid_1,
                                available_time, elevators, segment_index
                            )
                            if path_info:
                                unique_eid_key = f"{eid}|{eid_1}"
                                path_results[unique_eid_key] = path_info

            # 楼内运输或单电梯跨楼
            building_num1 = get_building_from_node(start)
            building_num2 = get_building_from_node(target)

            if eid.split("_")[0] == building_num1 or eid.split("_")[0] == building_num2:
                res = g.dijkstra_extra(start, target)
                if not res or "total_time" not in res or not res["path"]:
                    continue

                path_info = _build_single_elevator_path_info(
                    res, eid, available_time, elevators,
                    segment_index, start, target
                )
                if path_info:
                    path_results[eid] = path_info

        if actual_time:
            available_time = actual_time + available_time
        segment_index += 1
        start = pickup_pos
        target = target_pos

    if not path_results:
        print(f"[!] Task {tid} failed: No valid path from {start_pos} to {target_pos}")
        return None, None, {}

    # 选择最优路径
    pick_paths = {k: v for k, v in path_results.items() if v["status"] == PathPhase.PICK.value}
    deliver_paths = {k: v for k, v in path_results.items() if v["status"] == PathPhase.DELIVER.value}

    best_pick_info = None
    best_pick_key = None
    if pick_paths:
        best_pick_key = min(pick_paths.keys(), key=lambda k: pick_paths[k]["actual_time"])
        best_pick_info = path_results[best_pick_key]

    best_deliver_info = None
    best_deliver_key = None
    if deliver_paths:
        best_deliver_key = min(deliver_paths.keys(), key=lambda k: deliver_paths[k]["actual_time"])
        best_deliver_info = path_results[best_deliver_key]

    return best_pick_info, best_deliver_info, path_results


def _build_two_elevator_path_info(
    res_1: Dict[str, Any],
    res_2: Dict[str, Any],
    eid: str,
    eid_1: str,
    available_time: float,
    elevators: Dict[str, Elevator],
    status: int
) -> Optional[PathInfo]:
    """构建两部电梯路径信息"""
    from src.core.node import show_path_with_coords

    before = res_1["segments"]["before"]
    between_1 = res_1["segments"]["between"]
    transfer = res_2["segments"]["before"]
    between = res_2["segments"]["between"]
    after = res_2["segments"]["after"]

    start_e1, end_e1 = res_1["E_nodes"]
    start_e2, end_e2 = res_2["E_nodes"]

    path = merge_paths(res_1["path"], res_2["path"])

    if not start_e1 or not end_e1 or math.isinf(res_1["total_time"]) or \
       not start_e2 or not end_e2 or math.isinf(res_2["total_time"]):
        return None

    elev_1 = elevators[eid]
    elev_2 = elevators[eid_1]

    from_floor_1 = int(start_e1.split("_")[0])
    end_floor_1 = int(end_e1.split("_")[0])
    from_floor_2 = int(start_e2.split("_")[0])
    end_floor_2 = int(end_e2.split("_")[0])

    # 检查第一部电梯可预约性
    elev_start_1, elev_end_1, elev_ready_1 = elev_1.check_reserve(
        available_time, before, between_1, from_floor_1
    )
    wait_time_1 = max(elev_ready_1 - available_time - before, 0)
    part_time_1 = elev_end_1 - available_time

    # 检查第二部电梯可预约性
    elev_start_2, elev_end_2, elev_ready_2 = elev_2.check_reserve(
        elev_end_1, transfer, between, from_floor_2
    )
    wait_time_2 = max(elev_ready_2 - elev_end_1 - transfer, 0)
    part_time_2 = elev_end_2 + after - elev_end_1
    actual_time = elev_end_2 + after - available_time

    return {
        "path": path,
        "path_1": res_1["path"],
        "path_2": res_2["path"],
        "start_time": available_time,
        "actual_time": actual_time,
        "part_time_1": part_time_1,
        "part_time_2": part_time_2,
        "wait_time_1": wait_time_1,
        "wait_time_2": wait_time_2,
        "before": before,
        "between_1": between_1,
        "transfer": transfer,
        "between": between,
        "after": after,
        "start_e1": start_e1,
        "end_e1": end_e1,
        "start_e2": start_e2,
        "end_e2": end_e2,
        "eid_1": eid,
        "eid_2": eid_1,
        "type": "elevator",
        "from_floor_1": from_floor_1,
        "end_floor_1": end_floor_1,
        "from_floor_2": from_floor_2,
        "end_floor_2": end_floor_2,
        "elev_start_1": elev_start_1,
        "elev_end_1": elev_end_1,
        "elev_start_2": elev_start_2,
        "elev_end_2": elev_end_2,
        "status": status
    }


def _build_single_elevator_path_info(
    res: Dict[str, Any],
    eid: str,
    available_time: float,
    elevators: Dict[str, Elevator],
    status: int,
    start: str,
    target: str
) -> Optional[PathInfo]:
    """构建单电梯路径信息"""
    before = res["segments"]["before"]
    between = res["segments"]["between"]
    after = res["segments"]["after"]
    start_e2, end_e2 = res["E_nodes"]

    if not start_e2 or not end_e2 or math.isinf(res["total_time"]):
        return None

    elev_2 = elevators[eid]
    from_floor_2 = int(start_e2.split("_")[0])
    end_floor_2 = int(end_e2.split("_")[0])

    elev_start_2, elev_end_2, elev_ready_2 = elev_2.check_reserve(
        available_time, before, between, from_floor_2
    )
    wait_time_2 = max(elev_ready_2 - available_time - before, 0)
    actual_time = elev_end_2 + after - available_time

    return {
        "path": res["path"],
        "start_time": available_time,
        "actual_time": actual_time,
        "part_time_1": 0,
        "part_time_2": actual_time,
        "wait_time_1": 0,
        "wait_time_2": wait_time_2,
        "before": before,
        "between_1": 0,
        "transfer": 0,
        "between": between,
        "after": after,
        "start_e1": None,
        "end_e1": None,
        "start_e2": start_e2,
        "end_e2": end_e2,
        "eid_1": None,
        "eid_2": eid,
        "type": "elevator",
        "from_floor_1": 0,
        "end_floor_1": 0,
        "from_floor_2": from_floor_2,
        "end_floor_2": end_floor_2,
        "elev_start_1": 0,
        "elev_end_1": 0,
        "elev_start_2": elev_start_2,
        "elev_end_2": elev_end_2,
        "status": status,
        "elevator_stair": needs_two_elevators(start, target)
    }


def has_stairs(path_info: PathInfo) -> bool:
    """
    判断路径是否包含楼梯

    Args:
        path_info: 路径信息

    Returns:
        是否包含楼梯
    """
    if path_info.get("type") == "stair":
        return True

    path = path_info.get("path", [])
    for node in path:
        if "stair" in node.lower():
            return True
    return False