"""
楼梯冲突检测与解决模块

提供楼梯使用冲突检测和解决功能。
冲突类型包括：同向超车、双向会车、交叉路口。
"""

import copy
from collections import defaultdict
from typing import Dict, List, Tuple, Optional, Any

from src.models.path_info import PathInfo, Assignment
from src.utils.constants import PATH_PHASE_KEYS, PathPhase
from src.utils.config import DEFAULT_CONFIG, SystemConfig
from src.utils.logger import scheduler_logger


class StairConflictType:
    """楼梯冲突类型"""
    SAME_DIRECTION = "same_direction"      # 同向超车
    OPPOSITE_DIRECTION = "opposite_direction"  # 双向会车
    INTERSECTION = "intersection"          # 交叉路口


class StairConflictResolver:
    """
    楼梯冲突检测与解决类

    检测楼梯使用的时间重叠和方向冲突，
    提供基于优先级的解决策略。
    """

    def __init__(self, config: SystemConfig = DEFAULT_CONFIG):
        self.config = config

    def detect_stair_conflicts(self, assignments: List[Assignment]) -> List[Dict]:
        """
        检测楼梯使用冲突

        Args:
            assignments: 任务分配列表

        Returns:
            冲突列表（按开始时间排序）
        """
        conflicts: List[Dict] = []
        stair_usages: Dict[str, List[Dict]] = defaultdict(list)

        # 收集所有楼梯使用信息
        for assignment in assignments:
            for phase in [PathPhase.PICK, PathPhase.DELIVER]:
                path_info_key = PATH_PHASE_KEYS[phase.value]
                path_info = assignment.get(path_info_key)

                if path_info:
                    self._collect_stair_usage(stair_usages, assignment, path_info, phase)

        # 检测每个楼梯的时间冲突
        for stair_id, usages in stair_usages.items():
            usages.sort(key=lambda x: x["time_window"][0])

            for i in range(len(usages)):
                for j in range(i + 1, len(usages)):
                    conflict = self._check_stair_conflict(usages[i], usages[j], stair_id)
                    if conflict:
                        conflicts.append(conflict)

        return sorted(conflicts, key=lambda x: x["start_time"])

    def _collect_stair_usage(
        self,
        stair_usages: Dict[str, List[Dict]],
        assignment: Assignment,
        path_info: PathInfo,
        phase: PathPhase
    ):
        """收集楼梯使用记录"""
        path = path_info.get("path", [])
        start_time = path_info.get("start_time", 0)
        actual_time = path_info.get("actual_time", 0)

        # 提取楼梯节点
        stair_nodes = self._extract_stair_nodes(path)

        for stair_node in stair_nodes:
            # 解析楼梯ID（格式：floor_building_StairX）
            stair_id = self._get_stair_id(stair_node)

            if stair_id:
                # 计算通过楼梯的时间窗口
                stair_start, stair_end = self._estimate_stair_time(
                    path, stair_node, start_time, actual_time
                )

                direction = self._get_direction(path, stair_node)

                stair_usages[stair_id].append({
                    "assignment": assignment,
                    "status": phase.value,
                    "stair_node": stair_node,
                    "time_window": (stair_start, stair_end),
                    "robot_id": assignment["robot_id"],
                    "direction": direction,
                    "path": path
                })

    def _extract_stair_nodes(self, path: List[str]) -> List[str]:
        """从路径中提取楼梯节点"""
        stair_nodes = []
        for node in path:
            if "stair" in node.lower() or "Stair" in node:
                stair_nodes.append(node)
        return stair_nodes

    def _get_stair_id(self, stair_node: str) -> Optional[str]:
        """从楼梯节点解析楼梯ID"""
        # 格式：floor_building_StairX_Y 或 floor_building_StairX
        parts = stair_node.split("_")
        if len(parts) >= 3:
            building = parts[1]
            stair_num = parts[2]  # Stair1, Stair2 等
            return f"{building}_{stair_num}"
        return None

    def _estimate_stair_time(
        self,
        path: List[str],
        stair_node: str,
        start_time: float,
        total_time: float
    ) -> Tuple[float, float]:
        """估算通过楼梯的时间窗口"""
        # 找到楼梯节点在路径中的位置
        try:
            stair_index = path.index(stair_node)
        except ValueError:
            return (start_time, start_time + total_time)

        # 简化估算：按路径节点数量均匀分配时间
        node_count = len(path)
        if node_count <= 1:
            return (start_time, start_time + total_time)

        time_per_node = total_time / node_count
        stair_start = start_time + stair_index * time_per_node
        # 楼梯节点本身需要一定时间（通常是相邻两个楼梯节点构成一段）
        stair_duration = time_per_node * 2  # 预估2个节点时间

        return (stair_start, stair_start + stair_duration)

    def _get_direction(self, path: List[str], stair_node: str) -> str:
        """判断机器人通过楼梯的方向"""
        try:
            idx = path.index(stair_node)
        except ValueError:
            return "unknown"

        # 根据前后节点楼层判断方向
        if idx > 0:
            prev_floor = self._get_floor(path[idx - 1])
        else:
            prev_floor = self._get_floor(stair_node)

        if idx < len(path) - 1:
            next_floor = self._get_floor(path[idx + 1])
        else:
            next_floor = self._get_floor(stair_node)

        if next_floor > prev_floor:
            return "up"
        elif next_floor < prev_floor:
            return "down"
        else:
            return "horizontal"

    def _get_floor(self, node: str) -> int:
        """从节点名称解析楼层"""
        parts = node.split("_")
        if parts:
            try:
                return int(parts[0])
            except ValueError:
                return 0
        return 0

    def _check_stair_conflict(
        self,
        usage1: Dict,
        usage2: Dict,
        stair_id: str
    ) -> Optional[Dict]:
        """检查两个楼梯使用是否冲突"""
        start1, end1 = usage1["time_window"]
        start2, end2 = usage2["time_window"]

        # 时间无重叠
        if end1 <= start2 or end2 <= start1:
            return None

        # 判断冲突类型
        direction1 = usage1.get("direction", "unknown")
        direction2 = usage2.get("direction", "unknown")

        conflict_type = self._determine_conflict_type(direction1, direction2)

        # 所有时间重叠都是潜在冲突
        return {
            "stair_id": stair_id,
            "conflict_type": conflict_type,
            "usage1": usage1,
            "usage2": usage2,
            "overlap_time": min(end1, end2) - max(start1, start2),
            "start_time": min(start1, start2)
        }

    def _determine_conflict_type(self, direction1: str, direction2: str) -> str:
        """根据方向判断冲突类型"""
        # 双向会车（最危险）
        if (direction1 == "up" and direction2 == "down") or \
           (direction1 == "down" and direction2 == "up"):
            return StairConflictType.OPPOSITE_DIRECTION

        # 同向超车
        if direction1 == direction2:
            return StairConflictType.SAME_DIRECTION

        # 其他情况（交叉路口）
        return StairConflictType.INTERSECTION

    def resolve_stair_conflicts(
        self,
        assignments: List[Assignment],
        conflicts: List[Dict]
    ) -> List[Assignment]:
        """
        解决楼梯冲突

        Args:
            assignments: 任务分配列表
            conflicts: 冲突列表

        Returns:
            解决冲突后的任务分配列表
        """
        resolved_assignments = copy.deepcopy(assignments)
        robot_to_assignment = {a["robot_id"]: a for a in resolved_assignments}

        for conflict in conflicts:
            usage1 = conflict["usage1"]
            usage2 = conflict["usage2"]
            conflict_type = conflict["conflict_type"]

            # 根据冲突类型选择解决策略
            winner_id, loser_id, delay_time = self._resolve_conflict_by_type(
                usage1, usage2, conflict_type, robot_to_assignment
            )

            scheduler_logger.info(
                f"楼梯 {conflict['stair_id']} {conflict_type} 冲突: "
                f"机器人 {winner_id} 获胜, 机器人 {loser_id} 延迟 {delay_time:.2f}s"
            )

            # 对失败方应用延迟
            if loser_id in robot_to_assignment:
                assignment = robot_to_assignment[loser_id]
                self._apply_delay(assignment, delay_time)

        return resolved_assignments

    def _resolve_conflict_by_type(
        self,
        usage1: Dict,
        usage2: Dict,
        conflict_type: str,
        robot_to_assignment: Dict[int, Assignment]
    ) -> Tuple[int, int, float]:
        """
        根据冲突类型解决冲突

        Returns:
            (获胜机器人ID, 失败机器人ID, 延迟时间)
        """
        robot_id1 = usage1["robot_id"]
        robot_id2 = usage2["robot_id"]

        # 获取任务优先级
        priority1 = self._get_task_priority(robot_id1, robot_to_assignment)
        priority2 = self._get_task_priority(robot_id2, robot_to_assignment)

        overlap_time = min(usage1["time_window"][1], usage2["time_window"][1]) - \
                       max(usage1["time_window"][0], usage2["time_window"][0])

        # 双向会车：必须一方等待
        if conflict_type == StairConflictType.OPPOSITE_DIRECTION:
            delay = overlap_time + self.config.status_update_interval
            if priority1 < priority2:
                return robot_id1, robot_id2, delay
            elif priority2 < priority1:
                return robot_id2, robot_id1, delay
            else:
                # 同优先级：先到达者获胜
                if usage1["time_window"][0] <= usage2["time_window"][0]:
                    return robot_id1, robot_id2, delay
                else:
                    return robot_id2, robot_id1, delay

        # 同向超车：速度快的通过，慢的等待
        elif conflict_type == StairConflictType.SAME_DIRECTION:
            # 简化处理：低优先级等待
            delay = self.config.status_update_interval
            if priority1 < priority2:
                return robot_id1, robot_id2, delay
            else:
                return robot_id2, robot_id1, delay

        # 交叉路口：按优先级等待
        else:
            delay = overlap_time
            if priority1 < priority2:
                return robot_id1, robot_id2, delay
            else:
                return robot_id2, robot_id1, delay

    def _get_task_priority(
        self,
        robot_id: int,
        robot_to_assignment: Dict[int, Assignment]
    ) -> int:
        """获取机器人任务的优先级"""
        assignment = robot_to_assignment.get(robot_id)
        if assignment:
            task = assignment.get("task")
            if task:
                return task.priority
        return 5  # 默认最低优先级

    def _apply_delay(self, assignment: Assignment, delay: float):
        """对任务分配应用时间延迟"""
        # 延迟开始时间
        current_start = assignment.get("start_time", 0)
        assignment["start_time"] = current_start + delay

        # 延迟结束时间
        current_end = assignment.get("end_time", 0)
        assignment["end_time"] = current_end + delay

        # 延迟路径信息
        for path_key in ["pick_path_info", "deliver_path_info"]:
            path_info = assignment.get(path_key)
            if path_info:
                current_path_start = path_info.get("start_time", 0)
                path_info["start_time"] = current_path_start + delay

                # 延迟电梯预约时间
                for elev_key in ["elev_start_1", "elev_end_1", "elev_start_2", "elev_end_2"]:
                    if elev_key in path_info and path_info[elev_key] > 0:
                        path_info[elev_key] = path_info[elev_key] + delay