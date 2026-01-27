# src/schedulers/batch_scheduler.py
"""
批量调度器模块（重构版）

作为协调器，整合路径选择、冲突解决和任务执行。
保留核心算法不变，提升代码可读性和可维护性。
"""

import json
import os
import time
import copy
from collections import defaultdict
from itertools import permutations, product
from typing import List, Dict, Tuple, Optional, Any

from src.core.graph import Graph
from src.core.graph_loader import load_stair_graph_and_elevator_graphs
from src.core.template_to_topo import building_graph
from src.core.node import show_path_with_coords, get_coordinates_from_node

from src.models.robot import Robot, RobotTaskInfo
from src.models.elevator import Elevator, Stair, init_elevators
from src.models.task import Task
from src.models.path_info import PathInfo, Assignment

from src.utils.config import DEFAULT_CONFIG, SystemConfig
from src.utils.constants import PATH_PHASE_KEYS, PathPhase
from src.utils.path_utils import merge_paths, has_stairs_in_path
from src.utils.position_utils import parse_position, needs_two_elevators
from src.utils.logger import scheduler_logger

from src.schedulers.path_selector import select_best_path_with_elevator, has_stairs
from src.schedulers.conflict_resolver import ConflictResolver
from src.schedulers.stair_conflict_resolver import StairConflictResolver


class BatchScheduler:
    """
    批量任务调度器

    作为协调器管理多机器人任务调度流程。
    核心算法保持不变：
    - Dijkstra 最短路径
    - 电梯预约冲突检测
    - 策略1/策略2 冲突解决

    Attributes:
        robots: 机器人列表
        elevators: 电梯字典
        stair_graph: 楼梯图
        elevator_graphs: 电梯增强图字典
        campus_name: 校园名称
        config: 系统配置
        conflict_resolver: 冲突解决器
        start_time: 系统启动时间
    """

    def __init__(
        self,
        robots: List[Robot],
        elevators: Dict[str, Elevator],
        stair_graph: Graph,
        elevator_graphs: Dict[str, Graph],
        campus_name: str,
        config: SystemConfig = DEFAULT_CONFIG
    ):
        self.robots = robots
        self.elevators = elevators
        self.stair_graph = stair_graph
        self.elevator_graphs = elevator_graphs
        self.campus_name = campus_name
        self.config = config
        self.conflict_resolver = ConflictResolver(elevators, config)
        self.stair_conflict_resolver = StairConflictResolver(config)
        self.start_time = time.time()
        self._robot_map: Dict[int, Robot] = {r.id: r for r in robots}  # 用于快速查找
        self._current_paths: Dict[int, Dict] = {}
        self._set_robot_start_time()

    def _set_robot_start_time(self):
        """设置所有机器人的初始时间"""
        for robot in self.robots:
            robot.set_initial_time(self.start_time)

    def schedule_batch(self, tasks: List[Task]) -> List[Assignment]:
        """
        批量调度主函数

        Args:
            tasks: 任务列表

        Returns:
            任务分配列表
        """
        current_time = time.time() - self.start_time
        min_tasks_total_time = float('inf')
        best_final_assignments: Optional[List[Assignment]] = None

        scheduler_logger.info(f"开始批量调度 {len(tasks)} 个任务")

        # 遍历所有可能的任务排列组合
        for perm_tasks in self._generate_task_permutations(tasks):
            # 初始任务分配
            initial_assignments = self._initial_assignment(perm_tasks, current_time)

            # 冲突检测与解决
            iteration = 0
            max_iterations = self.config.max_conflict_iterations
            final_assignments = initial_assignments.copy()

            while iteration < max_iterations:
                conflicts = self.conflict_resolver.detect_elevator_conflicts(final_assignments)

                if not conflicts:
                    scheduler_logger.debug("未检测到电梯冲突")
                    break

                scheduler_logger.info(f"检测到 {len(conflicts)} 个电梯冲突")
                final_assignments = self.conflict_resolver.resolve_conflicts(
                    final_assignments, conflicts, strategy="strategy2"
                )
                iteration += 1

            if iteration == max_iterations:
                scheduler_logger.warning(f"达到最大迭代次数 {max_iterations}")

            # 楼梯冲突解决
            final_assignments = self._resolve_stair_conflicts(final_assignments)

            # 记录最优解
            tasks_total_time = self._calculate_tasks_total_time(final_assignments)
            if tasks_total_time < min_tasks_total_time:
                min_tasks_total_time = tasks_total_time
                best_final_assignments = final_assignments

        # 执行调度
        if best_final_assignments:
            self._execute_assignments(best_final_assignments, current_time)

        return best_final_assignments or []

    def _generate_task_permutations(self, tasks: List[Task]):
        """生成任务排列组合（按优先级和技能分组）"""
        sorted_tasks = sorted(
            tasks,
            key=lambda x: (x.priority, 0 if x.skill == "dog" else 1)
        )

        grouped_tasks = defaultdict(list)
        for task in sorted_tasks:
            grouped_tasks[(task.priority, task.skill)].append(task)

        permuted_groups = []
        for group in grouped_tasks.values():
            if len(group) > 1:
                permuted_groups.append(permutations(group))
            else:
                permuted_groups.append([tuple(group)])

        for group_perms in product(*permuted_groups):
            flattened = []
            for perm in group_perms:
                flattened.extend(perm)
            yield flattened

    def _initial_assignment(
        self,
        tasks: List[Task],
        current_time: float
    ) -> List[Assignment]:
        """初始任务分配"""
        assignments: List[Assignment] = []

        # 重置机器人预期状态
        for robot in self.robots:
            robot.expected_position = robot.position
            robot.expected_available_time = robot.available_time
            robot.expected_charge = robot.charge

        for task in tasks:
            assignment = self._assign_single_task(task, current_time)
            if assignment:
                assignments.append(assignment)

        return assignments

    def _assign_single_task(
        self,
        task: Task,
        current_time: float
    ) -> Optional[Assignment]:
        """分配单个任务"""
        feasible_robots = self._find_feasible_robots(task)

        if not feasible_robots:
            scheduler_logger.warning(f"任务 {task.id} 没有匹配技能的机器人")
            return None

        best_robot: Optional[Robot] = None
        best_total_time = float('inf')
        best_pick_info: Optional[PathInfo] = None
        best_deliver_info: Optional[PathInfo] = None
        best_path_results: Dict[str, PathInfo] = {}
        early_start = float('inf')

        for robot in feasible_robots:
            available_time = max(current_time, robot.expected_available_time)

            pick_info, deliver_info, path_results = select_best_path_with_elevator(
                task=task,
                rid=robot.id,
                start_pos=robot.expected_position,
                pickup_pos=task.start,
                target_pos=task.target,
                stair_graph=self.stair_graph,
                graph_map=self.elevator_graphs,
                elevators=self.elevators,
                current_time=current_time,
                available_time=available_time,
                campus_name=self.campus_name,
                config=self.config
            )

            if not pick_info or not deliver_info:
                continue

            pick_total_time = pick_info["actual_time"]
            deliver_total_time = deliver_info["actual_time"]
            start_time = pick_info["start_time"]
            current_total_time = pick_total_time + deliver_total_time + start_time - current_time

            if current_total_time < best_total_time:
                best_robot = robot
                best_pick_info = copy.deepcopy(pick_info)
                best_deliver_info = copy.deepcopy(deliver_info)
                best_total_time = current_total_time
                best_path_results = copy.deepcopy(path_results)
                early_start = start_time
            elif current_total_time == best_total_time and start_time < early_start:
                best_robot = robot
                best_pick_info = copy.deepcopy(pick_info)
                best_deliver_info = copy.deepcopy(deliver_info)
                best_total_time = current_total_time
                best_path_results = copy.deepcopy(path_results)
                early_start = start_time

        if best_robot and best_pick_info and best_deliver_info:
            return self._build_assignment(
                task, best_robot,
                best_pick_info, best_deliver_info,
                best_path_results, current_time, best_total_time
            )

        return None

    def _build_assignment(
        self,
        task: Task,
        robot: Robot,
        pick_info: PathInfo,
        deliver_info: PathInfo,
        path_results: Dict[str, PathInfo],
        current_time: float,
        total_time: float
    ) -> Assignment:
        """构建任务分配"""
        final_start_time = max(current_time, robot.expected_available_time)

        assignment = {
            "task": task,
            "robot_id": robot.id,
            "pick_path_info": pick_info,
            "deliver_path_info": deliver_info,
            "path_results": path_results,
            "release_time": current_time,
            "start_time": final_start_time,
            "end_time": current_time + total_time,
            "selected": []
        }

        # 标记选中的路径
        for key, path_result in path_results.items():
            if pick_info == path_result or deliver_info == path_result:
                assignment["selected"].append(key)

        # 模拟预约电梯
        self.conflict_resolver._reserve_elevators_for_path(pick_info, robot.id, simulated=True)
        self.conflict_resolver._reserve_elevators_for_path(deliver_info, robot.id, simulated=True)

        # 更新机器人预期状态
        robot.expected_position = task.target
        robot.expected_available_time = final_start_time + total_time

        wait_robot = final_start_time - current_time
        scheduler_logger.info(
            f"任务 {task.id} 分配给机器人 {robot.id}, 预计时间: {total_time:.2f}s"
        )
        if wait_robot > 0:
            scheduler_logger.debug(f"需等待机器人: {wait_robot:.2f}s")

        return assignment

    def _find_feasible_robots(self, task: Task) -> List[Robot]:
        """查找技能匹配且电量充足的机器人"""
        return [
            r for r in self.robots
            if r.skill == task.skill
            and r.expected_charge >= self.config.min_charge_threshold
            and not r.is_charging
        ]

    def _resolve_stair_conflicts(self, assignments: List[Assignment]) -> List[Assignment]:
        """解决楼梯冲突"""
        # 检测楼梯冲突
        stair_conflicts = self.stair_conflict_resolver.detect_stair_conflicts(assignments)

        if not stair_conflicts:
            scheduler_logger.debug("未检测到楼梯冲突")
            return assignments

        scheduler_logger.info(f"检测到 {len(stair_conflicts)} 个楼梯冲突")

        # 解决楼梯冲突
        resolved_assignments = self.stair_conflict_resolver.resolve_stair_conflicts(
            assignments, stair_conflicts
        )

        return resolved_assignments

    def _calculate_tasks_total_time(self, assignments: List[Assignment]) -> float:
        """计算所有任务总耗时"""
        total = 0.0
        for assignment in assignments:
            pick_info = assignment.get("pick_path_info")
            deliver_info = assignment.get("deliver_path_info")
            release_time = assignment.get("release_time", 0)
            start_time = assignment.get("start_time", 0)

            pick_actual = pick_info.get("actual_time", 0) if pick_info else 0
            deliver_actual = deliver_info.get("actual_time", 0) if deliver_info else 0

            total += pick_actual + deliver_actual + start_time - release_time

        return total

    def _execute_assignments(
        self,
        assignments: List[Assignment],
        current_time: float
    ):
        """执行最终的任务分配"""
        scheduler_logger.info(f"执行任务分配")

        # 清空预约并重新预约
        self.conflict_resolver.clear_reservations()

        for assignment in assignments:
            robot = self._find_robot_by_id(assignment["robot_id"])
            task = assignment["task"]

            # 真实预约电梯
            pick_info = assignment.get("pick_path_info")
            deliver_info = assignment.get("deliver_path_info")

            if pick_info:
                self.conflict_resolver._reserve_elevators_for_path(
                    pick_info, robot.id, simulated=False
                )
            if deliver_info:
                self.conflict_resolver._reserve_elevators_for_path(
                    deliver_info, robot.id, simulated=False
                )

            # 更新机器人任务
            path1 = pick_info.get("path", []) if pick_info else []
            path2 = deliver_info.get("path", []) if deliver_info else []
            path_start_time = assignment["start_time"]

            pick_actual = pick_info.get("actual_time", 0) if pick_info else 0
            deliver_actual = deliver_info.get("actual_time", 0) if deliver_info else 0
            path_total_time = pick_actual + deliver_actual

            wait_time_1 = pick_info.get("wait_time_1", 0) if pick_info else 0
            wait_time_2 = pick_info.get("wait_time_2", 0) if pick_info else 0
            wait_time_3 = deliver_info.get("wait_time_1", 0) if deliver_info else 0
            wait_time_4 = deliver_info.get("wait_time_2", 0) if deliver_info else 0

            robot.add_task(
                task_id=task.id,
                start_time=path_start_time,
                finish_time=path_start_time + path_total_time,
                path1=path1,
                path2=path2,
                actual_time1=pick_actual,
                actual_time2=deliver_actual,
                wait_time_1=wait_time_1,
                wait_time_2=wait_time_2,
                wait_time_3=wait_time_3,
                wait_time_4=wait_time_4
            )

            robot.get_sorted_tasks()

            scheduler_logger.info(
                f"机器人 {robot.id} 执行任务 {task.id}: "
                f"{assignment['start_time']:.2f}s - {assignment['end_time']:.2f}s"
            )

    def _find_robot_by_id(self, robot_id: int) -> Optional[Robot]:
        """按 ID 查找机器人（使用字典快速查找）"""
        return self._robot_map.get(robot_id)

    def cancel_task(self, task_id: int) -> bool:
        """
        取消正在执行的任务

        Args:
            task_id: 任务ID

        Returns:
            是否成功取消
        """
        scheduler_logger.info(f"尝试取消任务 {task_id}")

        # 查找任务所属机器人
        target_robot: Optional[Robot] = None
        for robot in self.robots:
            if robot.get_task(task_id):
                target_robot = robot
                break

        if not target_robot:
            scheduler_logger.warning(f"任务 {task_id} 未找到")
            return False

        # 获取任务信息（用于释放电梯预约）
        task_info = target_robot.get_task(task_id)
        if not task_info:
            return False

        # 释放电梯预约
        self._release_elevator_reservations(task_id, target_robot.id)

        # 从机器人移除任务
        removed_task = target_robot.remove_task(task_id)
        if removed_task:
            scheduler_logger.info(
                f"任务 {task_id} 已从机器人 {target_robot.id} 移除"
            )
            return True

        return False

    def _release_elevator_reservations(self, task_id: int, robot_id: int):
        """释放任务相关的电梯预约"""
        for elevator_id, elevator in self.elevators.items():
            # 取消该机器人的所有预约
            cancelled = elevator.cancel_robot_reservations(robot_id)
            if cancelled > 0:
                scheduler_logger.info(
                    f"释放电梯 {elevator_id} 的 {cancelled} 个预约: robot={robot_id}"
                )

    def reschedule_after_cancellation(
        self,
        pending_tasks: List[Task],
        current_time: float = None
    ) -> List[Assignment]:
        """
        任务取消后重新调度待处理任务

        Args:
            pending_tasks: 待调度任务列表
            current_time: 当前时间（可选，默认使用系统时间）

        Returns:
            新的任务分配列表
        """
        if current_time is None:
            current_time = time.time() - self.start_time

        scheduler_logger.info(f"开始重调度 {len(pending_tasks)} 个待处理任务")

        # 重置机器人预期状态
        for robot in self.robots:
            with robot._lock:
                robot.expected_position = robot.position
                robot.expected_available_time = robot.available_time
                robot.expected_charge = robot.charge

        # 执行调度
        new_assignments = self.schedule_batch(pending_tasks)

        scheduler_logger.info(f"重调度完成，分配 {len(new_assignments)} 个任务")

        return new_assignments


def get_robot_status_real_time(
    batch_scheduler: BatchScheduler,
    current_timestamp: Optional[float] = None
) -> Dict[str, Any]:
    """
    获取当前所有机器人状态

    Args:
        batch_scheduler: 调度器实例
        current_timestamp: 当前时间戳（可选）

    Returns:
        状态数据字典
    """
    if current_timestamp is None:
        current_timestamp = time.time()

    now = current_timestamp - batch_scheduler.start_time

    data_list = []

    for r in batch_scheduler.robots:
        # 处理位置可能为 None 的情况
        pos = r.current_position
        if pos is None:
            pos_x, pos_y, pos_z = 0.0, 0.0, 0.0
        else:
            pos_x, pos_y, pos_z = pos

        status_val = 0 if now >= r.available_time else 1
        robot_type_val = 1 if r.skill.lower() == "dog" else 2
        robot_name = f"Dog{r.id}" if robot_type_val == 1 else f"Human{r.id}"

        data_list.append({
            "robotId": str(r.id),
            "robotName": robot_name,
            "robotType": robot_type_val,
            "status": status_val,
            "posionX": round(pos_x, 2),
            "posionY": round(pos_y, 2),
            "posionZ": round(pos_z, 2),
            "running_time": round(r.running_time, 2),
            "total_time": round(r.path_total_time, 2),
            "timeStamp": int(current_timestamp),
        })

    return {"dataList": data_list}


# ============================================================
# 交互式调度器（测试入口）
# ============================================================

BASE_PATH = os.path.dirname(os.path.dirname(__file__))
MERGED_YAML = os.path.join(BASE_PATH, "core", "merged_nodes.yaml")


def start_interactive_scheduler(campus_name: str = "sandun"):
    """
    交互式批量调度器

    用于测试和调试。
    """
    # 构建图
    building_graph(campus_name)

    # 加载图
    stair_graph, elevator_graphs = load_stair_graph_and_elevator_graphs(
        merged_nodes_yaml=os.path.join(BASE_PATH, "core", "data", campus_name, "merged_nodes.yaml"),
        elevator_xy_tol=DEFAULT_CONFIG.elevator_xy_tolerance,
        ensure_legacy_keys=True
    )

    # 初始化电梯
    elevator_names = list(elevator_graphs.keys())
    elevators = init_elevators(elevator_names)

    # 初始化机器人
    robots = [
        Robot(0, "dog", "4_1_p1", campus_name=campus_name),
        Robot(1, "dog", "4_1_p1", campus_name=campus_name),
        Robot(2, "human", "4_1_p1", campus_name=campus_name),
        Robot(3, "human", "4_1_p1", campus_name=campus_name),
    ]

    # 创建调度器
    scheduler = BatchScheduler(
        robots, elevators, stair_graph,
        elevator_graphs, campus_name
    )

    task_counter = 0

    print("=== 交互式批量调度系统 ===")
    print("输入 'batch <skill> <priority> <start> <target> ...' 调度任务")
    print("输入 'robot' 查看机器人状态")
    print("输入 'exit' 退出")

    while True:
        user_input = input("调度系统 > ").strip()

        if user_input.lower() == "exit":
            now = time.time() - scheduler.start_time
            print(f"\n系统运行时间: {now:.2f}秒")
            break

        elif user_input.lower() == "robot":
            now = time.time() - scheduler.start_time
            print(f"\n系统运行时间: {now:.2f}秒")
            status = get_robot_status_real_time(scheduler)
            print(json.dumps(status, indent=4, ensure_ascii=False))

        elif user_input.startswith("batch "):
            parts = user_input.split()[1:]
            if len(parts) % 4 != 0:
                print("格式错误: batch <skill> <priority> <start> <target> ...")
                continue

            batch_tasks = []
            for i in range(0, len(parts), 4):
                skill = parts[i]
                priority = int(parts[i + 1])
                start = parts[i + 2]
                target = parts[i + 3]
                batch_tasks.append(Task(task_counter, skill, start, target, priority))
                task_counter += 1

            assignments = scheduler.schedule_batch(batch_tasks)
            for a in assignments:
                print(f"任务 {a['task'].id} 分配给机器人 {a['robot_id']}")

        else:
            print("格式错误")


if __name__ == "__main__":
    start_interactive_scheduler()