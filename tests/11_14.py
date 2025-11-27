import time
from typing import Dict, List, Tuple
from src.core.graph import initial_six_graphs
from src.core.node import show_path_with_coords, get_coordinates_from_node, \
    get_xyz_from_path_and_time_with_elevator_wait
import math
import json
from collections import defaultdict

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
        # 新增：记录机器人的任务队列，用于批量调度
        self.task_queue = []

    def add_task_to_queue(self, task_schedule: Dict):
        """添加任务到队列"""
        self.task_queue.append(task_schedule)
        # 更新机器人的可用时间为该任务完成时间
        self.available_time = task_schedule['completion_time']
        self.position = task_schedule['target_position']


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
    批量任务调度器，重点解决电梯冲突问题
    核心思想：一次性考虑所有任务和所有资源约束，找到全局最优解
    """

    def __init__(self, robots, elevators, stair_graph, elevator_graphs):
        self.robots = robots
        self.elevators = elevators
        self.stair_graph = stair_graph
        self.elevator_graphs = elevator_graphs

    def batch_assign_tasks(self, tasks: List[Task], current_time: float) -> Dict:
        """
        批量分配任务的核心算法
        步骤：
        1. 为每个任务找到所有可行的机器人
        2. 计算所有可能的(任务, 机器人)对的代价（考虑电梯冲突）
        3. 使用冲突感知的分配算法找到全局最优解
        4. 执行分配并更新所有状态
        """
        print(f"\n=== 开始批量任务分配，共{len(tasks)}个任务 ===")

        # 清空机器人任务队列
        for robot in self.robots:
            robot.task_queue = []

        # 步骤1: 收集所有可行的任务-机器人对
        candidate_assignments = self._collect_candidate_assignments(tasks, current_time)

        if not candidate_assignments:
            return {"error": "没有可行的任务分配方案"}

        # 步骤2: 批量解决电梯冲突并计算实际代价
        conflict_aware_costs = self._resolve_elevator_conflicts_iterative(candidate_assignments, current_time)

        # 步骤3: 使用改进的分配算法找到最优解
        optimal_assignment = self._find_optimal_assignment(conflict_aware_costs, tasks)

        # 步骤4: 执行最终分配
        return self._execute_batch_assignment(optimal_assignment, current_time)

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
        使用贪心算法找到近似最优的任务分配
        优先分配代价最小的任务-机器人对
        """
        # 按任务分组候选方案
        task_candidates = defaultdict(list)
        for candidate in candidates:
            task_candidates[candidate['task'].id].append(candidate)

        # 贪心分配：每次选择代价最小的可用分配
        assignment = {}
        used_robots = set()
        assigned_tasks = set()

        # 按代价排序所有候选对
        all_candidates = sorted(candidates, key=lambda x: x['final_cost'])

        for candidate in all_candidates:
            task = candidate['task']
            robot = candidate['robot']

            if task.id not in assigned_tasks and robot.id not in used_robots:
                assignment[task.id] = candidate
                used_robots.add(robot.id)
                assigned_tasks.add(task.id)

                # 如果所有任务都已分配，提前结束
                if len(assigned_tasks) == len(tasks):
                    break

        return assignment

    def _execute_batch_assignment(self, assignment: Dict, current_time: float) -> Dict:
        """执行最终的批量分配"""
        results = {}

        for task_id, assignment_info in assignment.items():
            task = assignment_info['task']
            robot = assignment_info['robot']
            path_info = assignment_info['best_path']

            # 更新机器人状态
            robot.position = task.target
            robot.path = path_info['path']
            robot.path_start_time = current_time
            robot.path_total_time = path_info['actual_time']
            robot.available_time = current_time + path_info['actual_time']
            robot.wait_time = path_info.get('wait_time', 0.0)
            robot.current_position = get_coordinates_from_node(robot.position)

            # 记录路径信息
            real_path = show_path_with_coords(path_info['path'])
            current_paths[task.id] = {
                "route": assignment_info['path_type'],
                "path": path_info['path'],
                "real_path": real_path,
                "total_time": path_info['actual_time'],
                "wait_time": path_info.get('wait_time', 0.0)
            }

            # 如果是电梯路径，进行最终预约
            if path_info['type'] == 'elevator':
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
            results[task.id] = {
                "robot_id": robot.id,
                "task_id": task.id,
                "start_time": current_time,
                "end_time": robot.available_time,
                "path_type": assignment_info['path_type'],
                "total_time": path_info['actual_time']
            }

            print(
                f"任务 {task.id} 分配给机器人 {robot.id}, 路径: {assignment_info['path_type']}, 总时间: {path_info['actual_time']:.2f}s")

        return {"success": True, "assignments": results}


# ============================================================
# 原有函数保持不变，新增批量处理接口
# ============================================================
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


# ============================================================
# 增强的终端接口 - 支持批量任务输入
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
    print("示例: batch dog 6_3_G human 4_3_A dog 5_2_B")
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
            parts = user_input.split()[1:]  # 去掉"batch"
            if len(parts) % 2 != 0:
                print("错误: 批量任务参数必须成对出现 (skill target)")
                continue

            # 解析批量任务
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
                for task_id, assignment in result["assignments"].items():
                    print(f"  任务{task_id}: 机器人{assignment['robot_id']}, 时间{assignment['total_time']:.2f}s")
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

            # 使用单任务分配（原有逻辑）
            # ... 原有单任务分配逻辑 ...

            task_counter += 1


if __name__ == "__main__":
    start_batch_scheduler()