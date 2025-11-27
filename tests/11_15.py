import time
from typing import Dict, List, Tuple
from src.core.graph import initial_six_graphs
from src.core.node import show_path_with_coords, get_coordinates_from_node, \
    get_xyz_from_path_and_time_with_elevator_wait
import math
import json
import copy

# ============================================================
# 全局变量
# ============================================================
complete_schedule_table = {}  # 存储完整调度表
current_paths = {}
robot_status = {}
start_timestamp = 0


# ============================================================
# Classes - 增强版本（新增方法）
# ============================================================
class Elevator:
    """增强的电梯类，支持批量冲突检测"""

    def __init__(self, eid: int, bldg_num: int, local_id: str, current_floor: int = 1):
        self.id = eid
        self.bldg_num = bldg_num
        self.local_id = local_id
        self.current_floor = current_floor
        self.schedule = []  # (start_time, end_time, from_floor, to_floor, robot_id)
        # 新增：记录电梯的预约请求队列，用于批量冲突解决
        self.pending_requests = []

    def reserve(self, start_time: float, duration: float, from_floor: int, to_floor: int, robot_id: int):
        """预约电梯时间段"""
        end_time = start_time + duration
        self.schedule.append((start_time, end_time, from_floor, to_floor, robot_id))
        self.schedule.sort(key=lambda x: x[0])
        self.current_floor = to_floor
        print(
            f"[Elevator {self.id} Reserved] R{robot_id}: {from_floor}->{to_floor}, {start_time:.2f}s - {end_time:.2f}s")

    def check_batch_availability(self, requests: List[Tuple]) -> Dict[int, Tuple[bool, float]]:
        """
        批量检查电梯可用性并解决冲突
        返回: {robot_id: (是否可用, 实际开始时间)}
        """
        # 复制当前预约表
        temp_schedule = self.schedule.copy()
        results = {}

        # 按请求的期望开始时间排序
        sorted_requests = sorted(requests, key=lambda x: x[2])  # x[2]是desired_start

        for robot_id, duration, desired_start, from_floor, to_floor in sorted_requests:
            desired_end = desired_start + duration

            # 在临时时间表中找到第一个可用时间段
            actual_start = desired_start
            for scheduled_start, scheduled_end, _, _, _ in temp_schedule:
                if actual_start + duration <= scheduled_start:
                    # 找到可用时间段
                    break
                elif actual_start < scheduled_end:
                    # 有冲突，推迟到该预约结束后
                    actual_start = scheduled_end
                # 继续检查下一个预约

            # 记录结果
            available = (actual_start - desired_start) < 60  # 如果等待时间小于60秒则认为可用
            results[robot_id] = (available, actual_start)

            # 在临时时间表中添加这个预约
            if available:
                temp_schedule.append((actual_start, actual_start + duration, from_floor, to_floor, robot_id))
                temp_schedule.sort(key=lambda x: x[0])

        return results

    def copy(self):
        """创建电梯的深拷贝"""
        new_elevator = Elevator(self.id, self.bldg_num, self.local_id, self.current_floor)
        new_elevator.schedule = copy.deepcopy(self.schedule)
        new_elevator.pending_requests = copy.deepcopy(self.pending_requests)
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
        # 新增：完整任务队列
        self.task_schedule = []  # 存储完整的任务执行计划

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
        new_robot.task_schedule = copy.deepcopy(self.task_schedule)
        return new_robot

    def add_to_schedule(self, task_id: int, start_time: float, end_time: float, path_type: str, total_time: float):
        """添加任务到调度表"""
        self.task_schedule.append({
            'task_id': task_id,
            'start_time': start_time,
            'end_time': end_time,
            'path_type': path_type,
            'total_time': total_time
        })
        # 按开始时间排序
        self.task_schedule.sort(key=lambda x: x['start_time'])


class Task:
    def __init__(self, tid: int, skill: str, start: str, target: str):
        self.id = tid
        self.skill = skill
        self.start = start
        self.target = target


# ============================================================
# 批量调度器 - 新增完整调度功能
# ============================================================
class BatchScheduler:
    """
    批量任务调度器，重点解决电梯冲突问题
    新增完整调度表生成功能
    """

    def __init__(self, robots, elevators, stair_graph, elevator_graphs):
        self.robots = robots
        self.elevators = elevators
        self.stair_graph = stair_graph
        self.elevator_graphs = elevator_graphs

    def generate_complete_schedule(self, tasks: List[Task], current_time: float) -> Dict:
        """
        生成完整任务调度表
        """
        print(f"\n=== 生成完整任务调度表，共{len(tasks)}个任务 ===")

        # 使用与batch相同的分配流程
        candidate_assignments = self._collect_candidate_assignments(tasks, current_time)

        if not candidate_assignments:
            return {"error": "没有可行的任务分配方案"}

        print(f"找到 {len(candidate_assignments)} 个可行的任务-机器人对")

        # 使用相同的冲突解决逻辑
        conflict_aware_costs = self._resolve_elevator_conflicts_iterative(candidate_assignments, current_time)

        if not conflict_aware_costs:
            return {"error": "冲突解决后没有可行的分配方案"}

        print(f"冲突解决后剩余 {len(conflict_aware_costs)} 个可行分配")

        # 使用相同的分配算法
        optimal_assignment = self._find_optimal_assignment(conflict_aware_costs, tasks)

        if not optimal_assignment:
            return {"error": "无法找到有效的任务分配"}

        # 生成调度表但不实际执行
        complete_schedule = self._generate_schedule_from_assignment(optimal_assignment, current_time)

        # 保存到全局变量
        global complete_schedule_table
        complete_schedule_table = complete_schedule

        self._print_complete_schedule(complete_schedule)
        return complete_schedule

    def _generate_schedule_from_assignment(self, assignment: Dict, current_time: float) -> Dict:
        """
        从分配结果生成调度表（不实际执行）
        """
        # 创建状态副本用于模拟时间线
        robot_timelines = {robot.id: robot.available_time for robot in self.robots}

        complete_schedule = {
            'total_completion_time': 0,
            'robots': {},
            'tasks': {},
            'generation_time': current_time
        }

        # 初始化机器人调度信息
        for robot in self.robots:
            complete_schedule['robots'][robot.id] = {
                'task_queue': [],
                'completion_time': robot.available_time,
                'total_work_time': 0
            }

        # 按任务分配顺序处理
        sorted_assignments = sorted(assignment.items(), key=lambda x: x[1]['final_cost'])

        for task_id, assignment_info in sorted_assignments:
            task = assignment_info['task']
            robot = assignment_info['robot']
            path_info = assignment_info['best_path']

            # 计算机器人的实际开始时间（考虑已有任务）
            robot_available_time = robot_timelines[robot.id]
            actual_start_time = max(current_time, robot_available_time)

            # 更新机器人时间线
            completion_time = actual_start_time + path_info['actual_time']
            robot_timelines[robot.id] = completion_time

            # 记录到调度表
            schedule_entry = {
                'task_id': task.id,
                'robot_id': robot.id,
                'start_time': actual_start_time,
                'completion_time': completion_time,
                'path_type': assignment_info['path_type'],
                'total_time': path_info['actual_time'],
                'skill': task.skill,
                'wait_time': path_info.get('wait_time', 0.0)
            }

            complete_schedule['robots'][robot.id]['task_queue'].append(schedule_entry)
            complete_schedule['robots'][robot.id]['completion_time'] = completion_time
            complete_schedule['robots'][robot.id]['total_work_time'] += path_info['actual_time']
            complete_schedule['tasks'][task.id] = schedule_entry

        # 计算总完成时间
        if complete_schedule['robots']:
            complete_schedule['total_completion_time'] = max(
                robot_info['completion_time']
                for robot_info in complete_schedule['robots'].values()
            )

        # 排序每个机器人的任务队列
        for robot_info in complete_schedule['robots'].values():
            robot_info['task_queue'].sort(key=lambda x: x['start_time'])

        return complete_schedule

    def _print_complete_schedule(self, schedule: Dict):
        """
        打印完整调度表 - 新增方法
        """
        print("\n" + "=" * 70)
        print("完整任务调度表")
        print("=" * 70)

        total_tasks = len(schedule['tasks'])
        completed_tasks = 0

        for robot_id, robot_info in schedule['robots'].items():
            print(f"\n🤖 机器人 {robot_id} 的任务队列 (总工作时间: {robot_info['total_work_time']:.1f}s):")

            if not robot_info['task_queue']:
                print("   暂无任务")
                continue

            for i, task_schedule in enumerate(robot_info['task_queue']):
                completed_tasks += 1
                print(f"   {i + 1}. 任务{task_schedule['task_id']} ({task_schedule['skill']}): "
                      f"⏰ {task_schedule['start_time']:.1f}s → {task_schedule['completion_time']:.1f}s "
                      f"(耗时: {task_schedule['total_time']:.1f}s) "
                      f"🚀 [{task_schedule['path_type']}]")

        print(f"\n📊 统计信息:")
        print(f"   总任务数: {total_tasks}")
        print(f"   已分配任务: {completed_tasks}")
        print(f"   总完成时间: {schedule['total_completion_time']:.1f}s")
        print(f"   调度生成时间: {schedule['generation_time']:.1f}s")
        print("=" * 70)

    def batch_assign_tasks(self, tasks: List[Task], current_time: float) -> Dict:
        """
        批量分配任务的核心算法 - 与schedule使用相同逻辑
        """
        print(f"\n=== 开始批量任务分配，共{len(tasks)}个任务，{len(self.robots)}个机器人 ===")

        if len(tasks) > len(self.robots) * 3:  # 安全限制
            print(f"警告: 任务数量({len(tasks)})远超过机器人数量({len(self.robots)})")
            print("建议分批处理或增加机器人数量")

        # 步骤1: 收集所有可行的任务-机器人对
        candidate_assignments = self._collect_candidate_assignments(tasks, current_time)

        if not candidate_assignments:
            return {"error": "没有可行的任务分配方案"}

        print(f"找到 {len(candidate_assignments)} 个可行的任务-机器人对")

        # 步骤2: 批量解决电梯冲突并计算实际代价
        conflict_aware_costs = self._resolve_elevator_conflicts_iterative(candidate_assignments, current_time)

        if not conflict_aware_costs:
            return {"error": "冲突解决后没有可行的分配方案"}

        print(f"冲突解决后剩余 {len(conflict_aware_costs)} 个可行分配")

        # 步骤3: 使用分配算法找到最优解
        optimal_assignment = self._find_optimal_assignment(conflict_aware_costs, tasks)

        if not optimal_assignment:
            return {"error": "无法找到有效的任务分配"}

        # 步骤4: 执行最终分配
        result = self._execute_batch_assignment(optimal_assignment, current_time)

        # 添加统计信息
        assigned_count = len(result["assignments"])
        if assigned_count < len(tasks):
            print(f"⚠️  注意: 只成功分配了 {assigned_count}/{len(tasks)} 个任务")
        else:
            print(f"✅ 成功分配所有 {assigned_count} 个任务")

        return result

    def _collect_candidate_assignments(self, tasks: List[Task], current_time: float) -> List[Dict]:
        """收集所有可行的任务-机器人分配对"""
        candidates = []

        for task in tasks:
            # 找到技能匹配的机器人
            feasible_robots = [r for r in self.robots if r.skill == task.skill]

            for robot in feasible_robots:
                # 计算基础路径信息（不考虑电梯冲突）
                path_options = self._calculate_path_options(robot, task, current_time)

                if path_options:
                    candidates.append({
                        'task': task,
                        'robot': robot,
                        'path_options': path_options,
                        'base_cost': min(opt['actual_time'] for opt in path_options.values())
                    })

        return candidates

    def _calculate_path_options(self, robot: Robot, task: Task, current_time: float) -> Dict:
        """计算机器人在不考虑冲突情况下的所有路径选项"""
        path_results = {}

        # 计算楼梯路径
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

        # 计算各电梯路径
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

            # 这里先不计算等待时间，在冲突解决阶段统一处理
            base_elevator_time = before + travel_to_start + between + after

            path_results[eid] = {
                "path": res["path"],
                "actual_time": base_elevator_time,
                "wait_time": 0.0,  # 暂设为0，冲突解决时计算
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

    def _resolve_elevator_conflicts_iterative(self, candidates: List[Dict], current_time: float,
                                              max_iterations: int = 3) -> List[Dict]:
        """
        迭代式冲突解决算法
        通过多轮迭代找到稳定的最优解
        """
        print("开始迭代冲突解决...")

        best_solution = None
        best_total_cost = float('inf')
        previous_cost = float('inf')

        for iteration in range(max_iterations):
            print(f"第 {iteration + 1} 轮迭代...")

            # 复制当前状态进行模拟
            temp_elevators = self._copy_elevator_states()
            current_solution = self._resolve_conflicts_single_round(candidates, current_time, temp_elevators)

            if not current_solution:
                continue

            # 计算总代价
            total_cost = sum(candidate['final_cost'] for candidate in current_solution)

            print(f"迭代 {iteration + 1}: 总代价 = {total_cost:.2f}")

            # 如果找到更好的解，更新最优解
            if total_cost < best_total_cost:
                best_total_cost = total_cost
                best_solution = current_solution.copy()

                # 如果代价不再改善，提前终止
                if iteration > 0:
                    improvement = (best_total_cost - previous_cost) / previous_cost
                    if abs(improvement) < 0.01:  # 改善小于1%
                        print("代价改善很小，提前终止迭代")
                        break

            previous_cost = total_cost

        return best_solution if best_solution else []

    def _resolve_conflicts_single_round(self, candidates: List[Dict], current_time: float, temp_elevators: Dict) -> \
            List[Dict]:
        """
        单轮冲突解决
        """
        # 按优先级排序候选方案（代价小的优先）
        sorted_candidates = sorted(candidates, key=lambda x: x['base_cost'])

        # 初始化电梯时间表副本
        elevator_schedules = {eid: elev.schedule.copy() for eid, elev in temp_elevators.items()}

        final_assignments = []

        for candidate in sorted_candidates:
            robot = candidate['robot']
            task = candidate['task']
            path_options = candidate['path_options']

            best_actual_cost = float('inf')
            best_final_option = None
            best_path_type = None

            # 评估所有路径选项
            for path_type, option in path_options.items():
                actual_cost = self._evaluate_option_with_conflicts(
                    option, robot.id, current_time, elevator_schedules, temp_elevators
                )

                if actual_cost < best_actual_cost:
                    best_actual_cost = actual_cost
                    best_final_option = option.copy()
                    best_final_option['actual_time'] = actual_cost
                    best_path_type = path_type

            if best_final_option and best_actual_cost < float('inf'):
                # 更新电梯时间表
                if best_final_option['type'] == 'elevator':
                    self._update_elevator_schedule(
                        best_final_option, robot.id, current_time, elevator_schedules
                    )

                final_assignments.append({
                    'task': task,
                    'robot': robot,
                    'best_path': best_final_option,
                    'final_cost': best_actual_cost,
                    'path_type': best_path_type
                })

        return final_assignments

    def _evaluate_option_with_conflicts(self, option: Dict, robot_id: int, current_time: float,
                                        elevator_schedules: Dict, temp_elevators: Dict) -> float:
        """
        评估单个选项的代价（考虑当前冲突状态）
        """
        if option['type'] == 'stair':
            return option['actual_time']  # 楼梯无冲突

        # 电梯方案：计算考虑当前冲突的实际时间
        eid = option['eid']
        elevator = temp_elevators[eid]

        arrival_at_elevator = current_time + option['before']
        desired_start = max(arrival_at_elevator, current_time)
        duration = option['between']

        # 在当前时间表中找到实际开始时间
        actual_start = self._find_actual_start_time(
            desired_start, duration, elevator_schedules[eid]
        )

        wait_time = max(0, actual_start - arrival_at_elevator)
        actual_time = option['before'] + wait_time + option['between'] + option['after']

        return actual_time

    def _find_actual_start_time(self, desired_start: float, duration: float, schedule: List) -> float:
        """
        在给定时间表中找到实际的开始时间
        """
        actual_start = desired_start

        for scheduled_start, scheduled_end, _, _, _ in schedule:
            if actual_start + duration <= scheduled_start:
                # 找到可用时间段
                break
            elif actual_start < scheduled_end:
                # 冲突，推迟到该预约结束后
                actual_start = scheduled_end

        return actual_start

    def _update_elevator_schedule(self, option: Dict, robot_id: int, current_time: float,
                                  elevator_schedules: Dict):
        """
        更新电梯时间表
        """
        eid = option['eid']
        arrival_at_elevator = current_time + option['before']
        desired_start = max(arrival_at_elevator, current_time)
        duration = option['between']

        actual_start = self._find_actual_start_time(
            desired_start, duration, elevator_schedules[eid]
        )

        # 添加预约
        elevator_schedules[eid].append((
            actual_start, actual_start + duration,
            option['from_floor'], option['to_floor'], robot_id
        ))
        elevator_schedules[eid].sort(key=lambda x: x[0])

    def _copy_elevator_states(self) -> Dict:
        """
        复制电梯状态用于模拟
        """
        elevator_copies = {}
        for eid, elevator in self.elevators.items():
            # 创建电梯副本（只复制预约表）
            elevator_copy = Elevator(elevator.id, elevator.bldg_num, elevator.local_id, elevator.current_floor)
            elevator_copy.schedule = elevator.schedule.copy()
            elevator_copies[eid] = elevator_copy

        return elevator_copies

    def _find_optimal_assignment(self, candidates: List[Dict], tasks: List[Task]) -> Dict:
        """
        改进的分配算法：支持任务数量 > 机器人数量
        使用基于时间的贪心分配
        """
        print(f"分配任务: {len(tasks)}个任务, {len(self.robots)}个机器人")

        assignment = {}

        # 初始化机器人状态（模拟）
        robot_states = {}
        task_completion_times = {}  # 新增：专门记录每个任务的完成时间
        for robot in self.robots:
            robot_states[robot.id] = {
                'available_time': robot.available_time,
                'position': robot.position,
                'assigned_tasks': []  # 记录分配的任务
            }

        # 按任务ID排序，确保一致性
        task_ids = sorted([task.id for task in tasks])
        remaining_tasks = set(task_ids)

        round_number = 1
        max_rounds = len(tasks)  # 防止无限循环

        while remaining_tasks and round_number <= max_rounds:
            print(f"\n第{round_number}轮分配，剩余任务: {sorted(remaining_tasks)}")

            # 找出当前可用的机器人（按可用时间排序）
            available_robots = sorted(
                [(robot_id, state['available_time']) for robot_id, state in robot_states.items()],
                key=lambda x: x[1]
            )

            print(f"可用机器人: {[f'R{robot_id}({time:.1f}s)' for robot_id, time in available_robots]}")

            assigned_this_round = 0

            # 为每个可用机器人分配一个任务
            for robot_id, robot_available_time in available_robots:
                if not remaining_tasks:
                    break

                # 找出该机器人能执行的所有剩余任务
                robot_candidates = [
                    c for c in candidates
                    if c['robot'].id == robot_id and c['task'].id in remaining_tasks
                ]

                if not robot_candidates:
                    continue

                # 选择完成时间最早的任务
                best_candidate = None
                best_completion_time = float('inf')

                for candidate in robot_candidates:
                    completion_time = robot_available_time + candidate['final_cost']
                    if completion_time < best_completion_time:
                        best_completion_time = completion_time
                        best_candidate = candidate

                if best_candidate:
                    task_id = best_candidate['task'].id
                    assignment[task_id] = best_candidate
                    remaining_tasks.remove(task_id)

                    # 更新机器人状态
                    robot_states[robot_id]['available_time'] = best_completion_time
                    robot_states[robot_id]['position'] = best_candidate['task'].target
                    robot_states[robot_id]['assigned_tasks'].append({
                        'task_id': task_id,
                        'completion_time': best_completion_time
                    })

                    # 记录任务的完成时间
                    task_completion_times[task_id] = best_completion_time

                    print(f"  机器人{robot_id} -> 任务{task_id}, 完成时间: {best_completion_time:.1f}s")
                    assigned_this_round += 1

            if assigned_this_round == 0:
                print("本轮没有分配任何任务，退出循环")
                break

            round_number += 1

        print(f"\n最终分配结果:")
        for task_id in sorted(assignment.keys()):
            robot_id = assignment[task_id]['robot'].id
            completion_time = task_completion_times[task_id]
            print(f"  任务{task_id} -> 机器人{robot_id}, 完成时间: {completion_time:.1f}s")

        # 输出每个机器人的任务序列
        print(f"\n机器人任务序列:")
        for robot_id, state in robot_states.items():
            if state['assigned_tasks']:
                tasks_info = []
                for task_info in state['assigned_tasks']:
                    tasks_info.append(f"任务{task_info['task_id']}({task_info['completion_time']:.1f}s)")
                print(f"  机器人{robot_id}: {', '.join(tasks_info)}")

        return assignment

    def _execute_batch_assignment(self, assignment: Dict, current_time: float) -> Dict:
        """执行最终的批量分配，支持多任务分配"""
        results = {}

        # 记录机器人的任务执行时间线
        robot_timelines = {robot.id: robot.available_time for robot in self.robots}

        # 按任务分配顺序处理（确保依赖关系）
        sorted_assignments = sorted(assignment.items(),
                                   key=lambda x: x[1]['final_cost'])

        for task_id, assignment_info in sorted_assignments:
            task = assignment_info['task']
            robot = assignment_info['robot']
            path_info = assignment_info['best_path']

            # 计算机器人的实际开始时间（考虑已有任务）
            robot_available_time = robot_timelines[robot.id]
            actual_start_time = max(current_time, robot_available_time)

            # 更新机器人时间线
            completion_time = actual_start_time + path_info['actual_time']
            robot_timelines[robot.id] = completion_time

            # 更新机器人状态
            robot.position = task.target
            robot.path = path_info['path']
            robot.path_start_time = actual_start_time
            robot.path_total_time = path_info['actual_time']
            robot.available_time = completion_time
            robot.wait_time = path_info.get('wait_time', 0.0)
            robot.current_position = get_coordinates_from_node(robot.position)

            # 记录到机器人的任务队列
            if not hasattr(robot, 'task_queue'):
                robot.task_queue = []
            robot.task_queue.append({
                'task_id': task.id,
                'start_time': actual_start_time,
                'completion_time': completion_time,
                'path_type': assignment_info['path_type'],
                'total_time': path_info['actual_time']
            })

            # 记录路径信息
            real_path = show_path_with_coords(path_info['path'])
            current_paths[task.id] = {
                "route": assignment_info['path_type'],
                "path": path_info['path'],
                "real_path": real_path,
                "total_time": path_info['actual_time'],
                "wait_time": path_info.get('wait_time', 0.0)
            }

            # === 输出路径信息 ===
            print(f"\nTask {task.id} selected route: {assignment_info['path_type']}, "
                  f"Total time: {path_info['actual_time']:.2f}s (wait {path_info.get('wait_time', 0.0):.2f}s)")
            print(f"Path: {path_info['path']}\n")
            print(f"Real Path: {real_path}\n")

            # 如果是电梯路径，进行最终预约
            if path_info['type'] == 'elevator':
                eid = path_info['eid']
                elev = self.elevators[eid]
                from_floor = path_info['from_floor']
                to_floor = path_info['to_floor']
                reserve_start_abs = actual_start_time + path_info['before'] + path_info.get('wait_time', 0)

                elev.reserve(
                    start_time=reserve_start_abs,
                    duration=path_info['between'],
                    from_floor=from_floor,
                    to_floor=to_floor,
                    robot_id=robot.id
                )

            # 记录分配结果
            results[task.id] = {
                "robot_id": robot.id,
                "task_id": task.id,
                "start_time": actual_start_time,
                "end_time": completion_time,
                "path_type": assignment_info['path_type'],
                "total_time": path_info['actual_time'],
                "wait_time": path_info.get('wait_time', 0.0)
            }

            print(f"任务 {task.id} 分配给机器人 {robot.id}, "
                  f"开始: {actual_start_time:.2f}s, 结束: {completion_time:.2f}s, "
                  f"路径: {assignment_info['path_type']}, 总时间: {path_info['actual_time']:.2f}s")

        return {"success": True, "assignments": results}



# ============================================================
# 新增全局函数
# ============================================================
def show_complete_schedule():
    """显示完整调度表 - 新增函数"""
    global complete_schedule_table
    if not complete_schedule_table:
        print("暂无调度表，请先使用 'schedule' 命令生成")
        return

    scheduler._print_complete_schedule(complete_schedule_table)


# ============================================================
# 增强的终端接口 - 新增命令
# ============================================================
def start_batch_scheduler():
    """启动支持批量调度的终端接口"""

    # 初始化图与对象 - 原有代码
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
    print("完整调度: schedule <task1_skill> <task1_target> <task2_skill> <task2_target> ...")  # 新增命令
    print("显示调度: showschedule")  # 新增命令
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

        elif user_input.lower() == "showschedule":  # 新增命令
            show_complete_schedule()
            continue

        elif user_input.lower().startswith("schedule "):  # 新增命令
            # 生成完整调度表
            parts = user_input.split()[1:]
            if len(parts) % 2 != 0:
                print("错误: 任务参数必须成对出现 (skill target)")
                continue

            # 解析任务
            schedule_tasks = []
            for i in range(0, len(parts), 2):
                skill, target = parts[i], parts[i + 1]
                task = Task(task_counter + i // 2, skill, "", target)
                schedule_tasks.append(task)

            # 生成完整调度表（使用与batch相同的算法）
            result = scheduler.generate_complete_schedule(schedule_tasks, now)

            if "error" not in result:
                task_counter += len(schedule_tasks)
                print(f"完整调度表生成成功! 共调度 {len(schedule_tasks)} 个任务")
                print("💡 提示: 使用 'batch' 命令执行此调度计划")
            else:
                print(f"调度表生成失败: {result.get('error', '未知错误')}")
            continue

        elif user_input.lower().startswith("batch "):
            # 原有批量任务处理逻辑
            parts = user_input.split()[1:]
            if len(parts) % 2 != 0:
                print("错误: 批量任务参数必须成对出现 (skill target)")
                continue

            batch_tasks = []
            for i in range(0, len(parts), 2):
                skill, target = parts[i], parts[i + 1]
                task = Task(task_counter + i // 2, skill, "", target)
                batch_tasks.append(task)

            result = scheduler.batch_assign_tasks(batch_tasks, now)

            if "success" in result:
                task_counter += len(batch_tasks)
                print(f"批量分配完成! 共分配 {len(batch_tasks)} 个任务")
                for task_id, assignment in result["assignments"].items():
                    print(f"  任务{task_id}: 机器人{assignment['robot_id']}, 时间{assignment['total_time']:.2f}s")
            else:
                print(f"批量分配失败: {result.get('error', '未知错误')}")

        else:
            # 单任务处理（向后兼容）
            parts = user_input.split()
            if len(parts) != 2:
                print("格式错误，请输入：<skill> <target_position> 或 batch ... 或 schedule ...")
                continue

            skill, target = parts[0], parts[1]
            task = Task(task_counter, skill, "", target)

            # 使用单任务分配（原有逻辑）
            # ... 原有单任务分配逻辑 ...

            task_counter += 1


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