"""
冲突检测与解决模块

提供电梯使用冲突检测和解决功能。
"""

import copy
from collections import defaultdict, OrderedDict
from itertools import groupby
from operator import itemgetter
from typing import Dict, List, Tuple, Optional, Any

from src.models.elevator import Elevator
from src.models.path_info import PathInfo, Assignment, Conflict, ElevatorUsage, AlternativeRoute
from src.models.task import Task
from src.utils.constants import PATH_PHASE_KEYS, PathPhase
from src.utils.config import DEFAULT_CONFIG, SystemConfig
from src.utils.logger import scheduler_logger


class ConflictResolver:
    """
    冲突检测与解决类

    管理电梯冲突检测和解决逻辑。
    """

    def __init__(self, elevators: Dict[str, Elevator], config: SystemConfig = DEFAULT_CONFIG):
        self.elevators = elevators
        self.config = config
        self._simulated_reservations: Dict[str, List[Dict]] = defaultdict(list)

    def detect_elevator_conflicts(self, assignments: List[Assignment]) -> List[Conflict]:
        """
        检测电梯使用冲突

        Args:
            assignments: 任务分配列表

        Returns:
            冲突列表（按开始时间排序）
        """
        conflicts: List[Conflict] = []
        elevator_usage: Dict[str, List[ElevatorUsage]] = defaultdict(list)

        # 收集所有电梯使用信息
        for assignment in assignments:
            for phase in [PathPhase.PICK, PathPhase.DELIVER]:
                path_info_key = PATH_PHASE_KEYS[phase.value]
                path_info = assignment.get(path_info_key)

                if path_info and path_info.get("type") == "elevator":
                    self._collect_elevator_usage(elevator_usage, assignment, path_info, phase)

        # 检测每个电梯的时间冲突
        for elevator_id, usages in elevator_usage.items():
            usages.sort(key=lambda x: x["time_window"][0])

            for i in range(len(usages)):
                for j in range(i + 1, len(usages)):
                    conflict = self._check_usage_overlap(usages[i], usages[j], elevator_id)
                    if conflict:
                        conflicts.append(conflict)

        return sorted(conflicts, key=lambda x: x["start_time"])

    def _collect_elevator_usage(
        self,
        elevator_usage: Dict[str, List[ElevatorUsage]],
        assignment: Assignment,
        path_info: PathInfo,
        phase: PathPhase
    ):
        """收集电梯使用记录"""
        # 第一部电梯
        eid_1 = path_info.get("eid_1")
        if eid_1:
            elevator_usage[eid_1].append({
                "assignment": assignment,
                "status": phase.value,
                "part": 1,
                "time_window": (
                    path_info.get("elev_start_1", 0),
                    path_info.get("elev_end_1", 0)
                ),
                "robot_id": assignment["robot_id"]
            })

        # 第二部电梯
        eid_2 = path_info.get("eid_2")
        if eid_2:
            elevator_usage[eid_2].append({
                "assignment": assignment,
                "status": phase.value,
                "part": 2,
                "time_window": (
                    path_info.get("elev_start_2", 0),
                    path_info.get("elev_end_2", 0)
                ),
                "robot_id": assignment["robot_id"]
            })

    def _check_usage_overlap(
        self,
        usage1: ElevatorUsage,
        usage2: ElevatorUsage,
        elevator_id: str
    ) -> Optional[Conflict]:
        """检查两个使用记录是否重叠"""
        start1, end1 = usage1["time_window"]
        start2, end2 = usage2["time_window"]

        # 无冲突条件
        if end1 <= start2 or end2 <= start1:
            return None

        # 有冲突
        return {
            "elevator_id": elevator_id,
            "usage1": usage1,
            "usage2": usage2,
            "overlap_time": min(end1, end2) - max(start1, start2),
            "start_time": min(start1, start2)
        }

    def resolve_conflicts(
        self,
        assignments: List[Assignment],
        conflicts: List[Conflict],
        strategy: str = "strategy2"
    ) -> List[Assignment]:
        """
        解决电梯冲突

        Args:
            assignments: 任务分配列表
            conflicts: 冲突列表
            strategy: 解决策略 ("strategy1" 或 "strategy2")

        Returns:
            解决冲突后的任务分配列表
        """
        resolved_assignments = copy.deepcopy(assignments)
        robot_to_assignment = {a["robot_id"]: a for a in resolved_assignments}
        change_to_others = set()

        # 按电梯分组冲突
        get_elevator_id = itemgetter("elevator_id")
        conflicts_sorted = sorted(conflicts, key=get_elevator_id)

        for elevator_id, conflicts_in_elevator in groupby(conflicts_sorted, key=get_elevator_id):
            for conflict in conflicts_in_elevator:
                usage1 = conflict["usage1"]
                usage2 = conflict["usage2"]

                if strategy == "strategy1":
                    winner, loser, best_alternative = self._strategy1_priority(
                        usage1, usage2, elevator_id
                    )
                else:
                    winner, loser, best_alternative = self._strategy2_priority(
                        usage1, usage2, elevator_id
                    )

                winner_id = winner["robot_id"]
                loser_id = loser["robot_id"]

                if winner_id in change_to_others or loser_id in change_to_others:
                    continue

                new_elevator_id = best_alternative.get("elevator_id") if best_alternative else None
                if new_elevator_id == elevator_id:
                    new_elevator_id = "等待电梯空闲"
                elif not new_elevator_id:
                    new_elevator_id = "楼梯路径"

                scheduler_logger.info(
                    f"电梯 {elevator_id} 冲突: 机器人 {winner_id} 获胜, "
                    f"机器人 {loser_id} 路径调整为{new_elevator_id}"
                )

                if loser_id in robot_to_assignment and best_alternative:
                    assignment = robot_to_assignment[loser_id]
                    status = best_alternative["status"]
                    assignment[PATH_PHASE_KEYS[status]] = best_alternative["path_info"]
                    change_to_others.add(loser_id)

        # 重置预约并重新计算
        self._simulated_reservations.clear()
        self._recalculate_assignments(resolved_assignments)

        return resolved_assignments

    def _strategy1_priority(
        self,
        usage1: ElevatorUsage,
        usage2: ElevatorUsage,
        elevator_id: str
    ) -> Tuple[Dict, Dict, Optional[AlternativeRoute]]:
        """
        策略1: 最大化电梯利用率

        优先保持先到达的机器人使用电梯。
        """
        assignment1 = usage1["assignment"]
        assignment2 = usage2["assignment"]

        # 按时间决定 winner（先到达者优先）
        start1, end1 = usage1["time_window"]
        start2, end2 = usage2["time_window"]

        if start1 <= start2:
            return assignment1, assignment2, None
        else:
            return assignment2, assignment1, None

    def _strategy2_priority(
        self,
        usage1: ElevatorUsage,
        usage2: ElevatorUsage,
        elevator_id: str
    ) -> Tuple[Dict, Dict, Optional[AlternativeRoute]]:
        """
        策略2: 最小化代价增加

        选择代价增量小的机器人改变路径。
        """
        assignment1 = usage1["assignment"]
        assignment2 = usage2["assignment"]
        robot_id1 = usage1["robot_id"]
        robot_id2 = usage2["robot_id"]

        # 计算代价增量
        penalty1, alt1 = self._calculate_penalty_if_lose(assignment1, usage1, elevator_id, robot_id2)
        penalty2, alt2 = self._calculate_penalty_if_lose(assignment2, usage2, elevator_id, robot_id1)

        scheduler_logger.debug(f"策略2比较:")
        scheduler_logger.debug(f"  机器人{assignment1['robot_id']} 惩罚增量:{penalty1:.2f}s")
        scheduler_logger.debug(f"  机器人{assignment2['robot_id']} 惩罚增量:{penalty2:.2f}s")

        # 优先级判断
        task1: Task = assignment1["task"]
        task2: Task = assignment2["task"]

        if task1.priority > task2.priority:
            scheduler_logger.debug(f"  机器人{task2.id} 作为winner")
            return assignment2, assignment1, alt1
        elif task1.priority < task2.priority:
            scheduler_logger.debug(f"  机器人{task1.id} 作为winner")
            return assignment1, assignment2, alt2

        # 代价增量判断
        if penalty1 < penalty2 or (penalty1 == penalty2 and task1.id > task2.id):
            scheduler_logger.debug(f"  机器人{assignment2['robot_id']} 作为winner")
            return assignment2, assignment1, alt1
        else:
            scheduler_logger.debug(f"  机器人{assignment1['robot_id']} 作为winner")
            return assignment1, assignment2, alt2

    def _calculate_penalty_if_lose(
        self,
        assignment: Assignment,
        usage: ElevatorUsage,
        elevator_id: str,
        other_robot_id: int
    ) -> Tuple[float, Optional[AlternativeRoute]]:
        """计算竞争失败时的代价增加"""
        status = usage["status"]
        part = usage["part"]
        path_info_key = PATH_PHASE_KEYS[status]
        part_time_key = f"part_time_{part}"

        path_info = assignment.get(path_info_key)
        if not path_info:
            return float('inf'), None

        current_cost = path_info.get(part_time_key, float('inf'))

        # 查找替代路线（简化版：仅查找楼梯路径）
        alternatives = self._find_stair_alternatives(assignment, status, path_info)

        if not alternatives:
            return float('inf'), None

        best_alt = min(alternatives, key=lambda x: x.get("part_time_2", float('inf')))
        new_cost = best_alt.get("part_time_2", float('inf'))
        penalty = new_cost - current_cost

        return penalty, best_alt

    def _find_stair_alternatives(
        self,
        assignment: Assignment,
        status: int,
        path_info: PathInfo
    ) -> List[AlternativeRoute]:
        """查找楼梯替代路径"""
        alternatives: List[AlternativeRoute] = []
        path_results = assignment.get("path_results", {})

        # 楼梯路径
        stair_key = f"stair_{status + 1}"
        if stair_key in path_results:
            stair_route = path_results[stair_key]
            original_actual_time = path_info.get("actual_time", 0)

            alternatives.append({
                "type": "stair",
                "path_info": stair_route,
                "status": status,
                "part_time_1": stair_route.get("part_time_2", 0),
                "part_time_2": stair_route.get("part_time_2", 0),
                "original_actual_time": original_actual_time,
                "description": "楼梯路径",
                "from_cache": True
            })

        return alternatives

    def _recalculate_assignments(self, assignments: List[Assignment]):
        """重新计算分配的时间和预约"""
        for assignment in assignments:
            pick_info = assignment.get("pick_path_info")
            deliver_info = assignment.get("deliver_path_info")

            if pick_info:
                pick_actual = pick_info.get("part_time_1", 0) + pick_info.get("part_time_2", 0)
                pick_info["actual_time"] = pick_actual
                self._reserve_elevators_for_path(pick_info, assignment["robot_id"], simulated=True)

            if deliver_info:
                deliver_actual = deliver_info.get("part_time_1", 0) + deliver_info.get("part_time_2", 0)
                deliver_info["actual_time"] = deliver_actual
                self._reserve_elevators_for_path(deliver_info, assignment["robot_id"], simulated=True)

            start_time = assignment.get("start_time", 0)
            pick_actual = pick_info.get("actual_time", 0) if pick_info else 0
            deliver_actual = deliver_info.get("actual_time", 0) if deliver_info else 0
            assignment["end_time"] = start_time + pick_actual + deliver_actual

    def _reserve_elevators_for_path(
        self,
        path_info: PathInfo,
        robot_id: int,
        simulated: bool = True
    ):
        """为路径预约电梯"""
        if path_info.get("type") != "elevator":
            return

        # 第一部电梯
        eid_1 = path_info.get("eid_1")
        if eid_1:
            self._reserve_single_elevator(
                eid_1,
                path_info.get("elev_start_1", 0),
                path_info.get("elev_end_1", 0),
                path_info.get("start_e1", ""),
                path_info.get("end_e1", ""),
                robot_id,
                simulated
            )

        # 第二部电梯
        eid_2 = path_info.get("eid_2")
        if eid_2:
            self._reserve_single_elevator(
                eid_2,
                path_info.get("elev_start_2", 0),
                path_info.get("elev_end_2", 0),
                path_info.get("start_e2", ""),
                path_info.get("end_e2", ""),
                robot_id,
                simulated
            )

    def _reserve_single_elevator(
        self,
        elevator_id: str,
        start_time: float,
        end_time: float,
        start_node: str,
        end_node: str,
        robot_id: int,
        simulated: bool
    ):
        """预约单个电梯"""
        from_floor = int(start_node.split("_")[0]) if start_node else 0
        to_floor = int(end_node.split("_")[0]) if end_node else 0

        if simulated:
            self._simulated_reservations[elevator_id].append({
                "start_time": start_time,
                "end_time": end_time,
                "from_floor": from_floor,
                "to_floor": to_floor,
                "robot_id": robot_id
            })
        else:
            elevator = self.elevators.get(elevator_id)
            if elevator:
                elevator.reserve(start_time, end_time, from_floor, to_floor, robot_id)

    def clear_reservations(self):
        """清空模拟预约"""
        self._simulated_reservations.clear()

    def get_reservations(self) -> Dict[str, List[Dict]]:
        """获取当前预约状态"""
        return dict(self._simulated_reservations)