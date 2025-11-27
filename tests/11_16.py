import time
from typing import Dict, List, Tuple
from src.core.graph import initial_six_graphs
from src.core.node import show_path_with_coords, get_coordinates_from_node, \
    get_xyz_from_path_and_time_with_elevator_wait
import math
import json
from collections import defaultdict
import copy

# ============================================================
# 全局变量
# ============================================================
current_paths = {}
robot_status = {}
start_timestamp = 0


# ============================================================
# Classes - 增强版本
# ============================================================
class Elevator:
    def __init__(self, eid: int, bldg_num: int, local_id: str, current_floor: int = 1):
        self.id = eid
        self.bldg_num = bldg_num
        self.local_id = local_id
        self.current_floor = current_floor
        self.schedule = []  # (start_time, end_time, from_floor, to_floor, robot_id)

    def reserve(self, start_time: float, duration: float, from_floor: int, to_floor: int, robot_id: int):
        end_time = start_time + duration
        self.schedule.append((start_time, end_time, from_floor, to_floor, robot_id))
        self.schedule.sort(key=lambda x: x[0])
        self.current_floor = to_floor
        print(
            f"[Elevator {self.id} Reserved] R{robot_id}: {from_floor}->{to_floor}, {start_time:.2f}s - {end_time:.2f}s")

    def check_availability(self, desired_start: float, duration: float) -> Tuple[bool, float, float]:
        """
        检查电梯可用性，返回(是否可用, 实际开始时间, 等待时间)
        """
        desired_end = desired_start + duration

        if not self.schedule:
            return True, desired_start, 0.0

        # 找到第一个可用的时间段
        actual_start = desired_start
        for scheduled_start, scheduled_end, _, _, _ in self.schedule:
            if actual_start + duration <= scheduled_start:
                # 找到可用时间段
                break
            elif actual_start < scheduled_end:
                # 有冲突，推迟到该预约结束后
                actual_start = scheduled_end

        wait_time = max(0, actual_start - desired_start)
        return True, actual_start, wait_time

    def copy(self):
        """创建电梯的深拷贝"""
        new_elevator = Elevator(self.id, self.bldg_num, self.local_id, self.current_floor)
        new_elevator.schedule = copy.deepcopy(self.schedule)
        return new_elevator


class Robot:
    def __init__(self, rid: int, skill: str, position: str):
        self.id = rid
        self.skill = skill
        self.position = position
        self.available_time = 0.0
        self.path = []
        self.path_start_time = None
        self.path_total_time = 0.0
        self.running_time = 0.0
        self.wait_time = 0.0
        self.current_position = get_coordinates_from_node(position)

    def copy(self):
        """创建机器人的深拷贝"""
        new_robot = Robot(self.id, self.skill, self.position)
        new_robot.available_time = self.available_time
        new_robot.path = copy.deepcopy(self.path)
        new_robot.path_start_time = self.path_start_time
        new_robot.path_total_time = self.path_total_time
        new_robot.running_time = self.running_time
        new_robot.wait_time = self.wait_time
        new_robot.current_position = copy.deepcopy(self.current_position)
        return new_robot


class Task:
    def __init__(self, tid: int, skill: str, start: str, target: str):
        self.id = tid
        self.skill = skill
        self.start = start
        self.target = target


# ============================================================
# 批量调度器 - 核心新增组件
# ============================================================
class BatchScheduler:
    """
    批量任务调度器，重点解决电梯冲突和路径重选问题
    """

    def __init__(self, robots, elevators, stair_graph, elevator_graphs):
        self.robots = robots
        self.elevators = elevators
        self.stair_graph = stair_graph
        self.elevator_graphs = elevator_graphs

    def batch_assign_tasks(self, tasks: List[Task], current_time: float) -> Dict:
        """
        批量分配任务的核心算法
        支持电梯冲突检测和路径重选
        """
        print(f"\n=== 开始批量任务分配，共{len(tasks)}个任务，{len(self.robots)}个机器人 ===")

        # 步骤1: 为每个任务找到所有可行的路径选项
        all_path_options = self._collect_all_path_options(tasks, current_time)

        if not all_path_options:
            return {"error": "没有可行的任务分配方案"}

        print(f"收集到 {len(all_path_options)} 个任务的路径选项")

        # 步骤2: 迭代解决冲突和重选路径
        final_assignments = self._resolve_conflicts_and_assign(all_path_options, tasks, current_time)

        if not final_assignments:
            return {"error": "冲突解决后没有可行的分配方案"}

        # 步骤3: 执行最终分配
        result = self._execute_final_assignments(final_assignments, current_time)

        assigned_count = len(result["assignments"])
        if assigned_count < len(tasks):
            print(f"⚠️  注意: 只成功分配了 {assigned_count}/{len(tasks)} 个任务")
        else:
            print(f"✅ 成功分配所有 {assigned_count} 个任务")

        return result

    def _collect_all_path_options(self, tasks: List[Task], current_time: float) -> Dict[int, List[Dict]]:
        """
        为每个任务收集所有可行的路径选项
        返回: {task_id: [路径选项1, 路径选项2, ...]}
        """
        all_options = {}

        for task in tasks:
            task_options = []
            feasible_robots = [r for r in self.robots if r.skill == task.skill]

            for robot in feasible_robots:
                path_options = self._calculate_path_options(robot, task, current_time)
                if path_options:
                    # 为每个路径选项添加机器人信息
                    for path_type, option in path_options.items():
                        task_options.append({
                            'robot': robot,
                            'path_type': path_type,
                            'path_info': option,
                            'base_time': option['actual_time']  # 不考虑冲突的基础时间
                        })

            if task_options:
                # 按基础时间排序
                task_options.sort(key=lambda x: x['base_time'])
                all_options[task.id] = task_options

        return all_options

    def _calculate_path_options(self, robot: Robot, task: Task, current_time: float) -> Dict:
        """计算机器人的所有路径选项（不考虑冲突）"""
        path_results = {}

        # 楼梯路径
        path_stair, cost_stair = self.stair_graph.dijkstra(robot.position, task.target)
        if path_stair and not math.isinf(cost_stair):
            path_results["stair"] = {
                "path": path_stair,
                "actual_time": cost_stair,
                "wait_time": 0.0,
                "before": 0.0,
                "between": 0.0,
                "after": 0.0,
                "eid": None,
                "type": "stair"
            }

        # 电梯路径
        graph_map = {
            "1_E1": self.elevator_graphs["1_E1"],
            "1_E2": self.elevator_graphs["1_E2"],
            "2_E1": self.elevator_graphs["2_E1"],
            "2_E2": self.elevator_graphs["2_E2"],
            "3_E1": self.elevator_graphs["3_E1"],
            "3_E2": self.elevator_graphs["3_E2"],
        }

        for eid, g in graph_map.items():
            res = g.dijkstra_extra(robot.position, task.target)
            if not res or "total_time" not in res or not res["path"]:
                continue

            before = res["segments"]["before"]
            between = res["segments"]["between"]
            after = res["segments"]["after"]
            start_e, end_e = res["E_nodes"]

            if not start_e or not end_e or math.isinf(res["total_time"]):
                continue

            # 基础时间计算（不考虑冲突）
            elev = self.elevators[eid]
            from_floor = int(start_e.split("_")[0])
            travel_to_start = abs(elev.current_floor - from_floor) * 1.75

            base_elevator_time = before + travel_to_start + between + after

            path_results[eid] = {
                "path": res["path"],
                "actual_time": base_elevator_time,
                "wait_time": 0.0,
                "before": before,
                "between": between,
                "after": after,
                "start_e": start_e,
                "end_e": end_e,
                "eid": eid,
                "type": "elevator",
                "from_floor": from_floor,
                "to_floor": int(end_e.split("_")[0]),
                "elevator_travel_time": between
            }

        return path_results if path_results else None

    def _detect_elevator_conflicts(self, assignments: Dict, current_time: float, sim_elevators: Dict) -> List[Dict]:
        """
        检测电梯冲突
        返回冲突列表
        """
        conflicts = []

        # 按任务收集电梯使用信息
        elevator_usage = defaultdict(list)

        for task_id, assignment in assignments.items():
            path_info = assignment['path_info']
            if path_info['type'] == 'elevator':
                eid = path_info['eid']
                robot = assignment['robot']

                # 计算机器人到达电梯的时间
                arrival_at_elevator = current_time + path_info['before']
                desired_start = max(arrival_at_elevator, current_time)
                duration = path_info['between']

                elevator_usage[eid].append({
                    'task_id': task_id,
                    'robot_id': robot.id,
                    'desired_start': desired_start,
                    'duration': duration,
                    'desired_end': desired_start + duration
                })

        # 检测每个电梯的冲突
        for eid, usage_list in elevator_usage.items():
            if len(usage_list) <= 1:
                continue

            # 按期望开始时间排序
            sorted_usage = sorted(usage_list, key=lambda x: x['desired_start'])

            for i in range(len(sorted_usage) - 1):
                current = sorted_usage[i]
                next_usage = sorted_usage[i + 1]

                if current['desired_end'] > next_usage['desired_start']:
                    conflicts.append({
                        'elevator_id': eid,
                        'conflict_tasks': [current['task_id'], next_usage['task_id']],
                        'conflict_robots': [current['robot_id'], next_usage['robot_id']],
                        'time_overlap': current['desired_end'] - next_usage['desired_start']
                    })

        return conflicts

    def _resolve_conflicts_by_reselection(self, conflicts: List[Dict], assignments: Dict,
                                          all_path_options: Dict, current_time: float, sim_elevators: Dict) -> bool:
        """
        通过路径重选解决冲突
        返回是否成功更新
        """
        updated = False

        for conflict in conflicts:
            elevator_id = conflict['elevator_id']
            conflict_tasks = conflict['conflict_tasks']

            print(f"解决电梯{elevator_id}冲突: 任务{conflict_tasks}")

            # 为冲突的任务寻找替代路径
            for task_id in conflict_tasks:
                if task_id not in assignments:
                    continue

                current_assignment = assignments[task_id]
                current_path_type = current_assignment['path_type']

                # 如果当前就是电梯路径，尝试找替代路径
                if current_path_type != 'stair' and current_assignment['path_info']['eid'] == elevator_id:
                    alternatives = self._find_alternative_paths(
                        task_id, current_assignment, all_path_options, current_time, sim_elevators
                    )

                    if alternatives:
                        best_alternative = min(alternatives, key=lambda x: x['actual_time'])

                        if best_alternative['actual_time'] < current_assignment['path_info']['actual_time']:
                            # 更新为更好的路径
                            new_assignment = {
                                'robot': current_assignment['robot'],
                                'path_type': best_alternative['path_type'],
                                'path_info': best_alternative
                            }
                            assignments[task_id] = new_assignment
                            updated = True
                            print(f"  任务{task_id}: {current_path_type} -> {best_alternative['path_type']}, "
                                  f"时间: {current_assignment['path_info']['actual_time']:.1f}s -> {best_alternative['actual_time']:.1f}s")

        return updated

    def _resolve_conflicts_priority(self, all_conflicts: Dict, assignments: Dict,
                                    all_path_options: Dict, current_time: float, sim_elevators: Dict) -> bool:
        """
        改进的优先级解决冲突：短任务优先，相同则按任务ID
        """
        updated = False

        for elevator_id, conflict_groups in all_conflicts.items():
            for conflict_group in conflict_groups:
                task_ids = [req['task_id'] for req in conflict_group]
                print(f"解决电梯{elevator_id}冲突: 任务{task_ids}")

                # 改进的排序：先按持续时间，再按任务ID
                sorted_conflicts = sorted(conflict_group,
                                          key=lambda x: (x['duration'], x['task_id']))

                # 第一个任务获得电梯使用权
                winner = sorted_conflicts[0]
                winners = [winner]
                losers = sorted_conflicts[1:]

                print(f"  获胜任务: 任务{winner['task_id']}({winner['duration']:.1f}s)")
                print(f"  需调整任务: {[req['task_id'] for req in losers]}")

                # 为失败的任务寻找替代路径
                for loser in losers:
                    task_id = loser['task_id']
                    success = self._try_alternative_path(task_id, assignments,
                                                         all_path_options, current_time,
                                                         sim_elevators, elevator_id)
                    if success:
                        updated = True

        return updated

    def _resolve_conflicts_and_assign(self, all_path_options: Dict, tasks: List[Task], current_time: float) -> Dict:
        """
        改进的冲突解决算法：支持多任务同时冲突
        """
        print("开始冲突解决和路径重选...")

        # 创建状态副本用于模拟
        sim_elevators = {eid: elevator.copy() for eid, elevator in self.elevators.items()}

        # 初始分配：每个任务选择最优路径
        current_assignments = {}
        for task_id, options in all_path_options.items():
            if options:
                current_assignments[task_id] = options[0]  # 选择基础时间最短的

        max_iterations = 10
        for iteration in range(max_iterations):
            print(f"\n第 {iteration + 1} 轮冲突检测...")

            # 检测所有电梯冲突（改进版本）
            all_conflicts = self._detect_all_elevator_conflicts(current_assignments, current_time, sim_elevators)

            if not all_conflicts:
                print("✅ 未发现电梯冲突，分配完成")
                break

            print(f"发现 {len(all_conflicts)} 组电梯冲突")

            # 解决冲突：按优先级处理
            updated = self._resolve_conflicts_priority(all_conflicts, current_assignments,
                                                       all_path_options, current_time, sim_elevators)

            if not updated:
                print("⚠️ 无法解决所有冲突，尝试强制解决方案")
                # 强制将部分任务切换到楼梯
                self._force_resolve_conflicts(all_conflicts, current_assignments, all_path_options)
                break

            if iteration == max_iterations - 1:
                print("⚠️ 达到最大迭代次数，使用当前分配")

        return current_assignments

    def _detect_all_elevator_conflicts(self, assignments: Dict, current_time: float, sim_elevators: Dict) -> Dict[
        str, List]:
        """
        检测所有电梯的所有冲突
        返回: {elevator_id: [冲突任务列表]}
        """
        all_conflicts = defaultdict(list)

        # 按电梯分组所有使用请求
        elevator_requests = defaultdict(list)

        for task_id, assignment in assignments.items():
            path_info = assignment['path_info']
            if path_info['type'] == 'elevator':
                eid = path_info['eid']
                robot = assignment['robot']

                # 计算机器人期望使用电梯的时间段
                arrival_at_elevator = current_time + path_info['before']
                desired_start = max(arrival_at_elevator, current_time)
                duration = path_info['between']
                desired_end = desired_start + duration

                elevator_requests[eid].append({
                    'task_id': task_id,
                    'robot_id': robot.id,
                    'desired_start': desired_start,
                    'desired_end': desired_end,
                    'duration': duration,
                    'assignment': assignment
                })

        # 为每个电梯检测冲突
        for eid, requests in elevator_requests.items():
            if len(requests) <= 1:
                continue

            # 按期望开始时间排序
            sorted_requests = sorted(requests, key=lambda x: x['desired_start'])

            # 找出所有冲突的任务组
            conflict_group = []
            current_end = 0

            for i, req in enumerate(sorted_requests):
                if i == 0:
                    conflict_group = [req]
                    current_end = req['desired_end']
                    continue

                if req['desired_start'] < current_end:
                    # 与当前组冲突，加入组
                    conflict_group.append(req)
                    current_end = max(current_end, req['desired_end'])
                else:
                    # 不冲突，保存当前组并开始新组
                    if len(conflict_group) > 1:
                        all_conflicts[eid].append(conflict_group)
                    conflict_group = [req]
                    current_end = req['desired_end']

            # 处理最后一组
            if len(conflict_group) > 1:
                all_conflicts[eid].append(conflict_group)

        return all_conflicts

    def _find_best_alternative_path(self, task_id: int, current_assignment: Dict,
                                    all_path_options: Dict, current_time: float,
                                    sim_elevators: Dict, exclude_elevator: str) -> Dict:
        """
        寻找最佳替代路径（排除冲突的电梯）
        """
        task_options = all_path_options.get(task_id, [])
        best_alternative = None
        best_time = float('inf')

        for option in task_options:
            path_info = option['path_info']
            path_type = option['path_type']

            # 排除冲突的电梯
            if path_info['type'] == 'elevator' and path_info['eid'] == exclude_elevator:
                continue

            # 计算实际时间
            if path_info['type'] == 'elevator':
                actual_time = self._calculate_actual_elevator_time(
                    path_info, current_assignment['robot'].id, current_time, sim_elevators
                )
            else:
                actual_time = path_info['actual_time']  # 楼梯路径

            if actual_time < best_time:
                best_time = actual_time
                best_alternative = path_info.copy()
                best_alternative['actual_time'] = actual_time
                best_alternative['path_type'] = path_type

        return best_alternative

    def _force_resolve_conflicts(self, all_conflicts: Dict, assignments: Dict, all_path_options: Dict):
        """
        强制解决冲突：将部分任务切换到楼梯
        """
        for elevator_id, conflict_groups in all_conflicts.items():
            for conflict_group in conflict_groups:
                # 按任务持续时间排序，最长的任务强制切换到楼梯
                sorted_conflicts = sorted(conflict_group, key=lambda x: x['duration'], reverse=True)

                for i, conflict in enumerate(sorted_conflicts):
                    if i >= 1:  # 只保留一个任务使用电梯，其他强制切换
                        task_id = conflict['task_id']
                        task_options = all_path_options.get(task_id, [])

                        # 寻找楼梯路径
                        stair_option = None
                        for option in task_options:
                            if option['path_type'] == 'stair':
                                stair_option = option
                                break

                        if stair_option:
                            assignments[task_id] = stair_option
                            print(
                                f"  强制切换: 任务{task_id} -> 楼梯, 时间: {stair_option['path_info']['actual_time']:.1f}s")

    def _try_alternative_path(self, task_id: int, assignments: Dict,
                              all_path_options: Dict, current_time: float,
                              sim_elevators: Dict, exclude_elevator: str) -> bool:
        """
        改进的替代路径寻找：考虑等待、其他电梯、楼梯
        """
        current_assignment = assignments[task_id]
        current_path_type = current_assignment['path_type']
        current_time_cost = current_assignment['path_info']['actual_time']

        # 方案1: 等待当前电梯（计算实际等待时间）
        wait_alternative = self._calculate_wait_option(task_id, current_assignment,
                                                       current_time, sim_elevators)

        # 方案2: 切换到其他电梯
        elevator_alternatives = self._find_other_elevators(task_id, current_assignment,
                                                           all_path_options, current_time,
                                                           sim_elevators, exclude_elevator)

        # 方案3: 切换到楼梯
        stair_alternative = self._find_stair_option(task_id, all_path_options)

        # 比较所有方案
        all_alternatives = []
        if wait_alternative:
            all_alternatives.append(wait_alternative)
        if elevator_alternatives:
            all_alternatives.extend(elevator_alternatives)
        if stair_alternative:
            all_alternatives.append(stair_alternative)

        if not all_alternatives:
            return False

        # 选择时间最短的方案
        best_alternative = min(all_alternatives, key=lambda x: x['actual_time'])

        # 只有在新方案明显更好时才切换（避免来回震荡）
        improvement_threshold = 0.95  # 新方案至少比当前好5%
        if best_alternative['actual_time'] < current_time_cost * improvement_threshold:
            new_assignment = {
                'robot': current_assignment['robot'],
                'path_type': best_alternative['path_type'],
                'path_info': best_alternative
            }
            assignments[task_id] = new_assignment

            action = "等待" if best_alternative['path_type'].startswith("wait_") else "切换"
            print(f"  任务{task_id}: {current_path_type} -> {best_alternative['path_type']} ({action}), "
                  f"时间: {current_time_cost:.1f}s -> {best_alternative['actual_time']:.1f}s")
            return True
        else:
            print(f"  任务{task_id}: 保持当前路径{current_path_type}, 替代方案不够好")
            return False

    def _calculate_wait_option(self, task_id: int, current_assignment: Dict,
                               current_time: float, sim_elevators: Dict) -> Dict:
        """
        计算等待当前电梯的方案
        """
        path_info = current_assignment['path_info']
        if path_info['type'] != 'elevator':
            return None

        eid = path_info['eid']
        elevator = sim_elevators[eid]
        robot_id = current_assignment['robot'].id

        # 计算机器人到达电梯的时间
        arrival_at_elevator = current_time + path_info['before']
        desired_start = max(arrival_at_elevator, current_time)
        duration = path_info['between']

        # 计算实际开始时间和等待时间
        available, actual_start, wait_time = elevator.check_availability(desired_start, duration)

        if not available:
            return None

        # 计算实际时间（包含等待）
        actual_time = path_info['before'] + wait_time + path_info['between'] + path_info['after']

        wait_alternative = path_info.copy()
        wait_alternative['actual_time'] = actual_time
        wait_alternative['wait_time'] = wait_time
        wait_alternative['path_type'] = f"wait_{eid}"

        return wait_alternative

    def _find_other_elevators(self, task_id: int, current_assignment: Dict,
                              all_path_options: Dict, current_time: float,
                              sim_elevators: Dict, exclude_elevator: str) -> List[Dict]:
        """
        寻找其他电梯方案
        """
        task_options = all_path_options.get(task_id, [])
        alternatives = []

        for option in task_options:
            path_info = option['path_info']
            path_type = option['path_type']

            # 排除冲突的电梯和当前电梯
            if (path_info['type'] == 'elevator' and
                    path_info['eid'] != exclude_elevator and
                    path_type != current_assignment['path_type']):

                # 计算实际时间（考虑新电梯的冲突）
                actual_time = self._calculate_actual_elevator_time(
                    path_info, current_assignment['robot'].id, current_time, sim_elevators
                )

                if actual_time < float('inf'):
                    alternative = path_info.copy()
                    alternative['actual_time'] = actual_time
                    alternative['path_type'] = path_type
                    alternatives.append(alternative)

        return alternatives

    def _find_stair_option(self, task_id: int, all_path_options: Dict) -> Dict:
        """
        寻找楼梯方案
        """
        task_options = all_path_options.get(task_id, [])

        for option in task_options:
            if option['path_type'] == 'stair':
                stair_alternative = option['path_info'].copy()
                stair_alternative['path_type'] = 'stair'
                return stair_alternative

        return None

    def _find_alternative_paths(self, task_id: int, current_assignment: Dict,
                                all_path_options: Dict, current_time: float, sim_elevators: Dict) -> List[Dict]:
        """
        为任务寻找替代路径（考虑当前冲突状态）
        """
        alternatives = []
        task_options = all_path_options.get(task_id, [])

        for option in task_options:
            if option['path_type'] == current_assignment['path_type']:
                continue  # 跳过当前路径

            path_info = option['path_info']

            # 如果是电梯路径，计算实际时间（考虑冲突）
            if path_info['type'] == 'elevator':
                actual_time = self._calculate_actual_elevator_time(
                    path_info, current_assignment['robot'].id, current_time, sim_elevators
                )
            else:
                # 楼梯路径无冲突，使用基础时间
                actual_time = path_info['actual_time']

            if actual_time < float('inf'):
                alternative = path_info.copy()
                alternative['actual_time'] = actual_time
                alternative['path_type'] = option['path_type']
                alternatives.append(alternative)

        return alternatives

    def _calculate_actual_elevator_time(self, path_info: Dict, robot_id: int,
                                        current_time: float, sim_elevators: Dict) -> float:
        """
        计算电梯路径的实际时间（考虑冲突）
        """
        eid = path_info['eid']
        elevator = sim_elevators[eid]

        arrival_at_elevator = current_time + path_info['before']
        desired_start = max(arrival_at_elevator, current_time)
        duration = path_info['between']

        # 检查电梯可用性
        available, actual_start, wait_time = elevator.check_availability(desired_start, duration)

        if not available:
            return float('inf')

        # 计算实际时间
        actual_time = path_info['before'] + wait_time + path_info['between'] + path_info['after']
        return actual_time

    def _execute_final_assignments(self, assignments: Dict, current_time: float) -> Dict:
        """执行最终的分配"""
        results = {}

        for task_id in sorted(assignments.keys()):
            assignment = assignments[task_id]
            robot = assignment['robot']
            path_info = assignment['path_info']
            path_type = assignment['path_type']

            # 修复：使用真实的目标位置
            # 假设任务目标存储在某个地方，这里需要根据你的数据结构调整
            # 暂时使用机器人当前位置作为目标（实际应该从任务信息获取）
            robot.position = path_info['path'][-1] if path_info['path'] else robot.position

            # 更新机器人状态
            robot.path = path_info['path']
            robot.path_start_time = current_time
            robot.path_total_time = path_info['actual_time']
            robot.available_time = current_time + path_info['actual_time']
            robot.wait_time = path_info.get('wait_time', 0.0)
            robot.current_position = get_coordinates_from_node(robot.position)

            # 记录路径信息
            real_path = show_path_with_coords(path_info['path'])
            current_paths[task_id] = {
                "route": path_type,
                "path": path_info['path'],
                "real_path": real_path,
                "total_time": path_info['actual_time'],
                "wait_time": path_info.get('wait_time', 0.0)
            }

            # 输出路径信息
            print(f"\nTask {task_id} selected route: {path_type}, "
                  f"Total time: {path_info['actual_time']:.2f}s (wait {path_info.get('wait_time', 0.0):.2f}s)")
            print(f"Path: {path_info['path']}")
            print(f"Real Path: {real_path}")

            # 如果是电梯路径，进行最终预约
            if path_info['type'] == 'elevator' and not path_type.startswith("wait_"):
                eid = path_info['eid']
                elev = self.elevators[eid]
                from_floor = path_info['from_floor']
                to_floor = path_info['to_floor']
                reserve_start_abs = current_time + path_info['before'] + path_info.get('wait_time', 0)

                elev.reserve(
                    start_time=reserve_start_abs,
                    duration=path_info['between'],
                    from_floor=from_floor,
                    to_floor=to_floor,
                    robot_id=robot.id
                )

            # 记录分配结果
            results[task_id] = {
                "robot_id": robot.id,
                "task_id": task_id,
                "start_time": current_time,
                "end_time": robot.available_time,
                "path_type": path_type,
                "total_time": path_info['actual_time']
            }

            print(f"任务 {task_id} 分配给机器人 {robot.id}, 总时间: {path_info['actual_time']:.2f}s")

        return {"success": True, "assignments": results}

    # 原有的单任务分配方法保持不变
    def find_feasible_robots(self, task, current_time):
        return [r for r in self.robots if r.skill == task.skill]

    def assign_task(self, task, current_time):
        # ... 原有代码保持不变 ...
        pass


# ============================================================
# 增强的终端接口 - 支持批量任务
# ============================================================
def start_batch_scheduler():
    """启动支持批量调度的终端接口"""

    # 初始化图与对象
    stair_graph, add_1E1_graph, add_1E2_graph, add_2E1_graph, add_2E2_graph, add_3E1_graph, add_3E2_graph, _ = initial_six_graphs(
        speed_land=1.5, speed_stair=0.5
    )
    elevators = init_six_elevators()
    robots = [
        Robot(0, "dog", "1_1_Left_1"),
        Robot(1, "dog", "1_1_Left_1"),
        Robot(2, "human", "1_1_Left_1"),
        Robot(3, "human", "1_1_Left_1"),
    ]
    elevator_graphs = {
        "1_E1": add_1E1_graph, "1_E2": add_1E2_graph,
        "2_E1": add_2E1_graph, "2_E2": add_2E2_graph,
        "3_E1": add_3E1_graph, "3_E2": add_3E2_graph
    }

    global scheduler, start_timestamp
    scheduler = BatchScheduler(robots, elevators, stair_graph, elevator_graphs)
    start_timestamp = time.time()
    task_counter = 0

    print("=== 多机器人批量调度系统 ===")
    print("输入格式：")
    print("单任务: <skill> <target_position>")
    print("批量任务: batch <task1_skill> <task1_target> <task2_skill> <task2_target> ...")
    print("输入 'exit' 退出, 输入 'robot' 查看机器人状态")

    while True:
        user_input = input("Batch Scheduler> ").strip()
        now = time.time() - start_timestamp

        if user_input.lower() == "exit":
            print(f"\n系统运行时间: {now:.2f}s")
            break

        elif user_input.lower() == "robot":
            print(f"\n系统运行时间: {now:.2f}s")
            status_data = get_robot_status_real_time(current_timestamp=time.time())
            print(json.dumps(status_data, indent=4, ensure_ascii=False))
            continue

        elif user_input.lower().startswith("batch "):
            # 批量任务处理
            parts = user_input.split()[1:]
            if len(parts) % 2 != 0:
                print("错误: 批量任务参数必须成对出现 (skill target)")
                continue

            batch_tasks = []
            for i in range(0, len(parts), 2):
                skill, target = parts[i], parts[i + 1]
                task = Task(task_counter + i // 2, skill, "", target)
                batch_tasks.append(task)

            # 执行批量分配
            result = scheduler.batch_assign_tasks(batch_tasks, now)

            if "success" in result:
                task_counter += len(batch_tasks)
                print(f"批量分配完成! 共分配 {len(batch_tasks)} 个任务")
            else:
                print(f"批量分配失败: {result.get('error', '未知错误')}")

        else:
            # 单任务处理（向后兼容）
            parts = user_input.split()
            if len(parts) != 2:
                print("格式错误，请输入：<skill> <target_position> 或 batch ...")
                continue

            skill, target = parts[0], parts[1]
            task = Task(task_counter, skill, "", target)

            # 使用单任务分配
            result = scheduler.assign_task(task, now)

            if "error" in result:
                print(f"[!] {result['error']}")
            else:
                print(f"\nSchedule running time: {now:.2f}s")
                print(
                    f"[OK] 任务分配成功: Robot {result['robot_id']}, Start={result['start_time']:.2f}s, End={result['end_time']:.2f}s")
                task_counter += 1


# 其余原有函数保持不变
def init_six_elevators() -> Dict[str, Elevator]:
    elevators = {}
    elevators["1_E1"] = Elevator(1, 1, "E1")
    elevators["1_E2"] = Elevator(2, 1, "E2")
    elevators["2_E1"] = Elevator(3, 2, "E1")
    elevators["2_E2"] = Elevator(4, 2, "E2")
    elevators["3_E1"] = Elevator(5, 3, "E1")
    elevators["3_E2"] = Elevator(6, 3, "E2")
    return elevators


def get_robot_status_real_time(current_timestamp=None):
    """
        获取当前所有机器人状态，输出格式：
        posionX/Y/Z 是实时坐标
        增加 running_time 和 total_time
        """
    global scheduler, start_timestamp

    if current_timestamp is None:
        current_timestamp = time.time()

    # 使用相对时间 now，与终端 loop 一致
    now = current_timestamp - start_timestamp

    data_list = []

    for r in scheduler.robots:
        # 当前任务已运行时间
        if r.path_start_time is None:
            running_time = 0.0
        else:
            running_time = now - r.path_start_time
            if running_time < 0:
                running_time = 0.0
            elif running_time > r.path_total_time:
                running_time = r.path_total_time

        # 当前任务总运行时间
        total_time = r.path_total_time if r.path else 0.0

        # 获取实时坐标
        if r.path and len(r.path) > 0:
            try:
                pos_x, pos_y, pos_z = get_xyz_from_path_and_time_with_elevator_wait(
                    path_list=r.path,
                    t=running_time,
                    wait_time=r.wait_time
                )
            except Exception:
                pos_x, pos_y, pos_z = get_coordinates_from_node(r.position)
        else:
            pos_x, pos_y, pos_z = get_coordinates_from_node(r.position)

        # 状态判断
        status_val = 0 if now >= r.available_time else 1
        robot_type_val = 1 if r.skill.lower() == "dog" else 2
        robot_name = "Dog" + str(r.id) if robot_type_val == 1 else "Human" + str(r.id)

        data_list.append({
            "robotId": str(r.id),
            "robotName": robot_name,
            "robotType": robot_type_val,
            "status": status_val,
            "posionX": round(pos_x, 2),
            "posionY": round(pos_y, 2),
            "posionZ": round(pos_z, 2),
            "running_time": round(running_time, 2),  # 当前任务已运行时间
            "total_time": round(total_time, 2),  # 当前任务总运行时间
            "timeStamp": int(current_timestamp),
        })

    return {"dataList": data_list}


def input_task(user_input):
    if user_input != "":
        user_input = user_input.split(" ")
    else:
        user_input = input("New Task> ").strip()
    return user_input


if __name__ == "__main__":
    start_batch_scheduler()