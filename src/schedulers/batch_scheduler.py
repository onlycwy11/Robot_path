# batch_scheduler.py
import json
import time
import math
import itertools
from typing import List, Dict, Tuple, Any
from src.core.graph import initial_six_graphs
from src.core.node import show_path_with_coords, get_coordinates_from_node, \
    get_xyz_from_path_and_time_with_elevator_wait
from collections import defaultdict
import copy


# ============================================================
# 全局变量
# ============================================================
current_paths = {}  # 存储每个任务的路径信息
robot_status = {}  # 存储每个机器人的实时状态


class Elevator:
    def __init__(self, eid: int, bldg_num: int, local_id: str, initial_floor: int = 1):
        self.id = eid
        self.bldg_num = bldg_num  # 电梯所属大楼
        self.local_id = local_id
        self.initial_floor = initial_floor
        # 电梯调度表
        # 开始时间、结束时间、起始楼层、目标楼层、使用电梯的机器人ID
        self.schedule = []  # (start_time, end_time, from_floor, to_floor, robot_id)

    # 电梯当前楼层
    def get_current_floor(self, current_time: float) -> int:
        """
        根据当前时间和调度表，计算电梯的实际楼层
        """
        if not self.schedule:
            return self.initial_floor

        # 按开始时间排序
        sorted_sched = sorted(self.schedule, key=lambda x: x[0])

        for i, (s, e, from_floor, to_floor, robot_id) in enumerate(sorted_sched):
            if current_time < s:
                # 在当前预约开始前
                if i == 0:  # 在第一个预约开始前，电梯还在初始位置
                    return self.initial_floor
                else:
                    # 返回上一个预约结束后的位置
                    _, prev_e, _, prev_to, _ = sorted_sched[i - 1]
                    return prev_to

            elif s <= current_time <= e:
                # 在预约执行期间
                # 计算运行进度
                progress = (current_time - s) / (e - s)  # 0~1
                current_floor = from_floor + int(progress * (to_floor - from_floor))
                return current_floor

        # 在所有预约之后
        _, last_e, _, last_to, _ = sorted_sched[-1]
        return last_to

    # 电梯预约机制
    def reserve(self, start_time: float, duration: float, from_floor: int, to_floor: int, robot_id: int):
        end_time = start_time + duration
        self.schedule.append((start_time, end_time, from_floor, to_floor, robot_id))
        self.schedule.sort(key=lambda x: x[0])  # 按时间排序
        print(
            f"[Elevator {self.id} Reserved] R{robot_id}: {from_floor}->{to_floor}, {start_time:.2f}s - {end_time:.2f}s")


class Robot:
    def __init__(self, rid: int, skill: str, position: str):
        self.id = rid
        self.skill = skill
        self.position = position  # 节点名
        self.available_time = 0.0  # 秒表时间
        self.path = []  # 当前任务路径
        self.path_start_time = None  # 当前任务开始时间
        self.path_total_time = 0.0  # 当前任务总耗时
        self.running_time = 0.0  # 当前任务已执行时间
        self.wait_time = 0.0
        self.current_position = get_coordinates_from_node(position)  # 初始xyz坐标


class Task:
    def __init__(self, tid: int, skill: str, start: str, target: str, duration: float, priority: int = 0):
        self.id = tid
        self.skill = skill
        self.start = start
        self.target = target
        self.duration = duration
        self.priority = priority


def init_six_elevators() -> Dict[str, Elevator]:
    elevators = {}
    elevators["1_E1"] = Elevator(1, 1, "E1")
    elevators["1_E2"] = Elevator(2, 1, "E2")
    elevators["2_E1"] = Elevator(3, 2, "E1")
    elevators["2_E2"] = Elevator(4, 2, "E2")
    elevators["3_E1"] = Elevator(5, 3, "E1")
    elevators["3_E2"] = Elevator(6, 3, "E2")
    return elevators


# ============================================================
# Path Selection
# ============================================================
def select_best_path_with_elevator(
        tid: int,
        start_pos: str,
        target_pos: str,
        stair_graph,
        add_1E1_graph,
        add_1E2_graph,
        add_2E1_graph,
        add_2E2_graph,
        add_3E1_graph,
        add_3E2_graph,
        elevators: dict,
        current_time: float
):
    global current_paths  # 全局变量，存储所有任务的路径信息
    path_results = {}  # 存储所有可能路径的结果

    # 楼梯路径
    # 调用Dijkstra算法计算纯楼梯路径的最短路径和耗时
    path_stair, cost_stair = stair_graph.dijkstra(start_pos, target_pos)
    # 若找到有效路径且代价不是无穷大
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
    # 建立电梯ID到对应增强图的映射
    graph_map = {
        "1_E1": add_1E1_graph,
        "1_E2": add_1E2_graph,
        "2_E1": add_2E1_graph,
        "2_E2": add_2E2_graph,
        "3_E1": add_3E1_graph,
        "3_E2": add_3E2_graph,
    }

    for eid, g in graph_map.items():  # 遍历6部电梯
        res = g.dijkstra_extra(start_pos, target_pos)
        # 增强型的Dijkstra，返回包含电梯结点信息的详细结果

        # 若无结果或无路径，跳过
        if not res or "total_time" not in res or not res["path"]:
            continue

        before = res["segments"]["before"]  # 走到电梯的时间
        between = res["segments"]["between"]  # 电梯运行时间
        after = res["segments"]["after"]  # 出电梯到目标的时间
        start_e, end_e = res["E_nodes"]  # 电梯起点和终点

        if not start_e or not end_e or math.isinf(res["total_time"]):
            continue  # 如果电梯结点无效或代价无穷，跳过

        elev = elevators[eid]
        from_floor = int(start_e.split("_")[0])  # 从输入格式中提取起始楼层

        # 原思路：计算电梯到达起始楼层所需时间
        # elev.current_floor 是电梯的当前瞬时楼层
        # 电梯可能仍在为其他任务运行或等待，并非处于当前任务来临的初始状态
        # travel_to_start = abs(elev.current_floor - from_floor) * 1.75

        # ===========注意此处代码！！！仍存在问题！！===========
        # 机器人到达电梯前的时刻
        robot_arrival = current_time + before
        # 查询电梯在机器人到达时的预计位置
        elevator_position_at_arrival = elev.get_current_floor(robot_arrival)
        if elevator_position_at_arrival == from_floor:
            travel_to_start = 0.0
        else:
            travel_to_start = abs(elevator_position_at_arrival - from_floor) * 1.75
            # elevator_position_before_arrival = elev.get_current_floor(robot_arrival-travel_to_start)
            # if elevator_position_before_arrival != elevator_position_at_arrival:
        # 电梯到达起始楼层的时刻
        elevator_ready = current_time + travel_to_start
        # 机器人和电梯的 ”有效开始时间“ 较晚者，乘坐电梯需保证两者都就位
        effective_start = max(robot_arrival, elevator_ready)

        # 初始化等待时间
        wait_time = 0.0

        # 检查电梯调度表中的冲突
        # (开始时间s, 结束时间e, 起始楼层, 目标楼层, 机器人ID)
        for (s, e, _from, _to, _rid) in elev.schedule:
            # 如果当前预约时间段与已有预约冲突
            # 无冲突的情况：电梯停止时间早于预约开始时间 或 电梯启动时间晚于预约结束时间
            if not (effective_start + between <= s or effective_start >= e):
                # 计算需要等待的时间（直到冲突预约结束）
                wait_time = max(wait_time, e - effective_start)

        actual_time = before + travel_to_start + wait_time + between + after

        path_results[eid] = {
            "path": res["path"],
            "actual_time": actual_time,
            "wait_time": wait_time + travel_to_start,
            "before": before,
            "between": between,
            "after": after,
            "start_e": start_e,
            "end_e": end_e,
            "eid": eid,
            "type": "elevator"
        }

    if not path_results:
        print(f"[!] Task {tid} failed: No valid path found from {start_pos} to {target_pos}")
        return {"error": "no_valid_path"}

    best_key = min(path_results.keys(), key=lambda k: path_results[k]["actual_time"])
    best_info = path_results[best_key]
    real_path = show_path_with_coords(best_info["path"])

    current_paths[tid] = {
        "route": best_key,
        "path": best_info["path"],
        "real_path": real_path,
        "total_time": best_info['actual_time'],
        "wait_time": best_info['wait_time']
    }

    print(
        f"\nTask {tid} selected route: {best_key}, Total time: {best_info['actual_time']:.2f}s (wait {best_info['wait_time']:.2f}s)")
    print(f"Path: {best_info['path']}\n")
    print(f"Real Path: {real_path}\n")

    if best_info["type"] == "elevator":
        eid = best_info["eid"]
        elev = elevators[eid]
        from_floor = int(best_info["start_e"].split("_")[0])
        to_floor = int(best_info["end_e"].split("_")[0])
        reserve_start_abs = current_time + best_info["before"] + best_info["wait_time"]
        elev.reserve(  # 为最佳路径预约电梯
            start_time=reserve_start_abs,
            duration=best_info["between"],
            from_floor=from_floor,
            to_floor=to_floor,
            robot_id=tid
        )

    return best_info


class BatchScheduler:
    def __init__(self, robots, elevators, stair_graph, elevator_graphs):
        self.robots = robots
        self.elevators = elevators
        self.stair_graph = stair_graph
        self.elevator_graphs = elevator_graphs
        self.robot_plans = {}  # 存储机器人的任务计划
        self.elevator_schedules = defaultdict(list)  # 电梯调度表
        self.start_time = time.time()

    def schedule_batch(self, tasks: List[Task]):
        """
        批量调度主函数
        """
        current_time = time.time() - self.start_time

        print(f"\n=== 开始批量调度 {len(tasks)} 个任务 ===")

        # 1. 初始任务分配 - 为每个任务找到技能匹配且时间最优的机器人
        initial_assignments = self._initial_assignment(tasks, current_time)

        # 2. 检测电梯冲突
        conflicts = self._detect_elevator_conflicts(initial_assignments)

        if conflicts:
            print(f"检测到 {len(conflicts)} 个电梯冲突")
            # 3. 解决冲突（这里可以切换策略1或策略2）
            final_assignments = self._resolve_conflicts(initial_assignments, conflicts, strategy="strategy2")
        else:
            final_assignments = initial_assignments
            print("未检测到电梯冲突")

        # 4. 执行调度
        self._execute_assignments(final_assignments, current_time)

        return final_assignments

    def simulate_schedule(self, robots: List[Robot], task_order: List[Task]) -> float:
        """
        模拟调度过程，计算给定任务顺序的总完成时间（makespan）
        """
        # 初始化：记录每个机器人的空闲时间（初始为0，表示立即可用）
        robot_status = [r.available_time for r in robots]
        makespan = 0.0  # 总完成时间（所有机器人最后完成任务的时刻）

        # 遍历任务顺序中的每个任务
        for task in task_order:
            # 寻找最适合执行当前任务的机器人
            best_robot_idx = -1  # 标记是否找到可用机器人
            earliest_time = math.inf  # 记录最早可用时间

            # 检查所有机器人，找到技能匹配且最早可用的
            for i, robot in enumerate(robots):
                if robot.skill != task.skill:
                    continue  # 技能不匹配
                if robot_status[i] < earliest_time:
                    earliest_time = robot_status[i]  # 更新更早的可用时间
                    best_robot_idx = i  # 记录机器人索引

            # 如果没有可用机器人（理论上不存在）
            if best_robot_idx == -1:
                return math.inf  # 无可用机器人，调度失败

            # 分配任务给机器人
            robot_status[best_robot_idx] += task.duration
            makespan = max(makespan, robot_status[best_robot_idx])

        return makespan

    def find_optimal_schedule(self, robots: List[Robot], tasks: List[Task]) -> tuple[list[Any] | Any, float]:
        """
        遍历所有可能的任务顺序，找到总完成时间最短的组合
        """
        # 1. 按优先级分组（高优先级任务必须优先执行）
        tasks_sorted = sorted(tasks, key=lambda x: -x.priority)  # 降序排序
        priority_groups = {}
        for task in tasks_sorted:
            if task.priority not in priority_groups:
                priority_groups[task.priority] = []
            priority_groups[task.priority].append(task)

        # 2. 对每个优先级组内的任务生成排列组合
        optimal_order = []  # 存储当前最优任务顺序
        min_makespan = math.inf

        # 遍历优先级组（从高到低）
        for priority in sorted(priority_groups.keys(), reverse=True):
            group_tasks = priority_groups[priority]
            if not group_tasks:
                continue

            # 生成当前优先级组内所有可能的排列
            for perm in itertools.permutations(group_tasks):
                # 合并已确定的高优先级任务顺序
                current_order = optimal_order + list(perm)
                # 模拟调度
                current_makespan = self.simulate_schedule(robots, current_order)
                # 更新最优解
                if current_makespan < min_makespan:
                    min_makespan = current_makespan
                    optimal_order = current_order

        return optimal_order, min_makespan

    def _initial_assignment(self, tasks: List[Task], current_time: float) -> List[dict]:
        """
        初始任务分配：为每个任务选择最快到达的机器人
        """
        assignments = []

        for task in tasks:
            best_robot = None
            best_path_info = None
            best_total_time = float('inf')

            # 找到技能匹配的机器人
            feasible_robots = [r for r in self.robots if r.skill == task.skill]

            if not feasible_robots:
                print(f"[警告] 任务 {task.id} 没有匹配技能的机器人")
                continue

            for robot in feasible_robots:
                # 计算机器人到任务目标的最优路径
                path_info = select_best_path_with_elevator(
                    tid=task.id,
                    start_pos=robot.position,
                    target_pos=task.target,
                    stair_graph=self.stair_graph,
                    add_1E1_graph=self.elevator_graphs["1_E1"],
                    add_1E2_graph=self.elevator_graphs["1_E2"],
                    add_2E1_graph=self.elevator_graphs["2_E1"],
                    add_2E2_graph=self.elevator_graphs["2_E2"],
                    add_3E1_graph=self.elevator_graphs["3_E1"],
                    add_3E2_graph=self.elevator_graphs["3_E2"],
                    elevators=self.elevators,
                    current_time=max(current_time, robot.available_time)
                )

                if "error" in path_info:
                    continue

                total_time = path_info["actual_time"]
                if total_time < best_total_time:
                    best_robot = robot
                    best_path_info = path_info
                    best_total_time = total_time

            if best_robot and best_path_info:
                assignments.append({
                    "task": task,
                    "robot": best_robot,
                    "path_info": best_path_info,
                    "start_time": max(current_time, best_robot.available_time),
                    "end_time": max(current_time, best_robot.available_time) + best_total_time
                })
                print(f"任务 {task.id} 分配给机器人 {best_robot.id}, 预计时间: {best_total_time:.2f}s")

        return assignments

    def _detect_elevator_conflicts(self, assignments: List[dict]) -> List[dict]:
        """
        检测电梯使用冲突
        """
        conflicts = []
        elevator_usage = defaultdict(list)

        # 收集所有电梯使用信息
        for assignment in assignments:
            path_info = assignment["path_info"]
            if path_info["type"] == "elevator":
                elevator_id = path_info["eid"]
                start_time = assignment["start_time"] + path_info["before"]
                end_time = start_time + path_info["between"]

                elevator_usage[elevator_id].append({
                    "assignment": assignment,
                    "time_window": (start_time, end_time),
                    "robot_id": assignment["robot"].id
                })

        # 检测每个电梯的时间冲突
        for elevator_id, usages in elevator_usage.items():
            # 按开始时间排序
            usages.sort(key=lambda x: x["time_window"][0])

            # 检测重叠
            for i in range(len(usages)):
                for j in range(i + 1, len(usages)):
                    usage1 = usages[i]
                    usage2 = usages[j]

                    # 检查时间窗口是否重叠
                    start1, end1 = usage1["time_window"]
                    start2, end2 = usage2["time_window"]

                    if not (end1 <= start2 or end2 <= start1):
                        conflicts.append({
                            "elevator_id": elevator_id,
                            "usage1": usage1,
                            "usage2": usage2,
                            "overlap_time": min(end1, end2) - max(start1, start2)
                        })

        return conflicts

    def _resolve_conflicts(self, assignments: List[dict], conflicts: List[dict], strategy: str = "strategy2") -> List[
        dict]:
        """
        解决电梯冲突
        """
        # 创建可修改的副本
        resolved_assignments = copy.deepcopy(assignments)

        for conflict in conflicts:
            elevator_id = conflict["elevator_id"]
            usage1 = conflict["usage1"]
            usage2 = conflict["usage2"]

            assignment1 = usage1["assignment"]
            assignment2 = usage2["assignment"]

            if strategy == "strategy1":
                # 策略1: 最大化电梯利用率
                winner, loser = self._strategy1_priority(assignment1, assignment2, elevator_id)
            else:
                # 策略2: 最小化代价增加
                winner, loser = self._strategy2_priority(assignment1, assignment2, elevator_id)

            print(f"电梯 {elevator_id} 冲突: 机器人 {winner['robot'].id} 获胜, 机器人 {loser['robot'].id} 需要调整")

            # 处理失败方 - 这里调用学妹要实现的函数
            new_loser_assignment = self._handle_loser_robot(loser, elevator_id, resolved_assignments)

            # 更新分配
            resolved_assignments.remove(loser)
            resolved_assignments.append(new_loser_assignment)

        return resolved_assignments

    def _strategy1_priority(self, assignment1, assignment2, elevator_id):
        """
        策略1: 最大化电梯利用率
        """
        # 这里实现策略1的逻辑
        # 暂时返回assignment1为获胜者
        return assignment1, assignment2

    def _strategy2_priority(self, assignment1, assignment2, elevator_id):
        """
        策略2: 最小化代价增加
        """
        # 这里实现策略2的逻辑
        # 暂时返回assignment1为获胜者
        return assignment1, assignment2

    # 策略2相关
    def calculate_penalty_if_lose(assignment, elevator_id, current_assignments):
        """
        计算机器人竞争失败时的代价增加
        返回：代价增量（秒）
        """
        pass

    def find_alternative_routes(robot, task, forbidden_elevator_id, current_time):
        """
        查找不使用指定电梯的替代路线
        返回：替代路线列表，每条路线包含路径信息和时间
        """
        pass

    def calculate_wait_option(assignment, elevator_id, conflict_end_time):
        """
        计算等待电梯的方案
        返回：等待后的新assignment
        """
        pass

    def _handle_loser_robot(self, loser_assignment, elevator_id, current_assignments):
        """
        处理竞争失败的机器人 - 这是学妹需要实现的核心函数
        """
        # 这里应该实现：
        # 1. 计算等待电梯的代价
        # 2. 计算更换路线的代价
        # 3. 选择代价较小的方案

        # 临时实现：直接使用楼梯
        robot = loser_assignment["robot"]
        task = loser_assignment["task"]

        # 计算楼梯路径
        path_stair, cost_stair = self.stair_graph.dijkstra(robot.position, task.target)

        new_assignment = copy.deepcopy(loser_assignment)
        new_assignment["path_info"] = {
            "path": path_stair,
            "actual_time": cost_stair,
            "wait_time": 0.0,
            "type": "stair"
        }
        new_assignment["end_time"] = new_assignment["start_time"] + cost_stair

        print(f"机器人 {robot.id} 更换为楼梯路径, 时间: {cost_stair:.2f}s")

        return new_assignment

    def _execute_assignments(self, assignments: List[dict], current_time: float):
        """
        执行最终的任务分配
        """
        print(f"\n=== 执行任务分配 ===")

        for assignment in assignments:
            robot = assignment["robot"]
            task = assignment["task"]
            path_info = assignment["path_info"]

            # 更新机器人状态
            robot.position = task.target
            robot.path = path_info["path"]
            robot.path_start_time = assignment["start_time"]
            robot.path_total_time = path_info["actual_time"]
            robot.available_time = assignment["end_time"]
            robot.wait_time = path_info.get("wait_time", 0.0)
            robot.current_position = get_coordinates_from_node(robot.position)

            # 如果是电梯路径，预约电梯
            if path_info["type"] == "elevator":
                elevator_id = path_info["eid"]
                elevator = self.elevators[elevator_id]
                from_floor = int(path_info["start_e"].split("_")[0])
                to_floor = int(path_info["end_e"].split("_")[0])

                reserve_start = assignment["start_time"] + path_info["before"] + path_info.get("wait_time", 0)
                elevator.reserve(
                    start_time=reserve_start,
                    duration=path_info["between"],
                    from_floor=from_floor,
                    to_floor=to_floor,
                    robot_id=robot.id
                )

            print(
                f"机器人 {robot.id} 执行任务 {task.id}: {assignment['start_time']:.2f}s - {assignment['end_time']:.2f}s")

    def find_feasible_robots(self, task, current_time):
        return [r for r in self.robots if r.skill == task.skill]

    def assign_task(self, task, current_time):
        feasible_robots = self.find_feasible_robots(task, current_time)
        global robot_status

        if not feasible_robots:
            return {"error": f"No robot matches skill '{task.skill}' for Task {task.id}"}

        robot = min(feasible_robots, key=lambda r: r.available_time)
        if current_time < robot.available_time:
            return {
                "error": f"Task {task.id} failed: Robot {robot.id} busy until {robot.available_time - current_time:.2f}s later"}

        best_info = select_best_path_with_elevator(
            tid=robot.id,
            start_pos=robot.position,
            target_pos=task.target,
            stair_graph=self.stair_graph,
            add_1E1_graph=self.elevator_graphs["1_E1"],
            add_1E2_graph=self.elevator_graphs["1_E2"],
            add_2E1_graph=self.elevator_graphs["2_E1"],
            add_2E2_graph=self.elevator_graphs["2_E2"],
            add_3E1_graph=self.elevator_graphs["3_E1"],
            add_3E2_graph=self.elevator_graphs["3_E2"],
            elevators=self.elevators,
            current_time=current_time
        )

        if "error" in best_info:
            return {"error": f"Task {task.id} failed: No valid path from {robot.position} to {task.target}"}

        # 更新机器人状态
        robot.position = task.target
        robot.path = best_info["path"]
        robot.path_start_time = current_time
        robot.path_total_time = best_info["actual_time"]
        robot.available_time = current_time + best_info["actual_time"]
        robot.wait_time = best_info.get("wait_time", 0.0)

        robot.current_position = get_coordinates_from_node(robot.position)

        robot_status[robot.id] = {
            "position": robot.position,
            "real_position": robot.current_position,
            "available_time": robot.available_time,
            "current_task": task.id,
            "skill": robot.skill,
            "wait_time": robot.wait_time
        }

        return {
            "robot_id": robot.id,
            "task_id": task.id,
            "start_time": current_time,
            "end_time": robot.available_time,
            "path_info": best_info
        }


def get_robot_status_real_time(batch_scheduler, start_timestamp: int, current_timestamp=None):
    """
    获取当前所有机器人状态，输出格式：
    posionX/Y/Z 是实时坐标
    增加 running_time 和 total_time
    """
    if current_timestamp is None:
        current_timestamp = time.time()

    # 使用相对时间 now，与终端 loop 一致
    now = current_timestamp - start_timestamp

    data_list = []

    for r in batch_scheduler.robots:
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


# 批量调度使用示例
def batch_scheduling_demo():
    """
    批量调度演示函数
    """
    # 初始化（使用原有的初始化代码）
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

    # 创建批量调度器
    batch_scheduler = BatchScheduler(robots, elevators, stair_graph, elevator_graphs)

    # 创建批量任务
    tasks = [
        Task(0, "dog", "", "3_5_A", 1.2),
        Task(1, "human", "", "9_2_B", 2),
        Task(2, "dog", "", "6_3_C", 3),
        Task(3, "human", "", "3_7_D", 1)
    ]

    # 执行批量调度
    assignments = batch_scheduler.schedule_batch(tasks)

    return batch_scheduler, assignments


def start_interactive_scheduler():
    """
    交互式批量调度器
    用户可以动态输入任务，查看机器人状态，或退出系统
    """
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

    # 创建批量调度器
    batch_scheduler = BatchScheduler(robots, elevators, stair_graph, elevator_graphs)
    task_counter = 0

    print("=== 交互式批量调度系统 ===")
    print("输入任务格式：<skill> <target_position>，例如：dog 6_3_G")
    print("输入 'batch <skill> <target> <skill> <target> ...' 一次性调度多个任务")
    print("输入 'exit' 退出系统")
    print("输入 'robot' 查看机器人状态")
    print("----------------------------------")

    while True:
        now = time.time() - batch_scheduler.start_time
        prompt = "调度系统 > "
        user_input = input(prompt).strip()

        # 退出系统
        if user_input.lower() == "exit":
            print(f"\n系统运行时间: {now:.2f}秒")
            print("退出调度系统")
            break

        # 查看机器人状态
        elif user_input.lower() == "robot":
            print(f"\n系统运行时间: {now:.2f}秒")
            print("\n--- 机器人状态 ---")
            status_data = get_robot_status_real_time(current_timestamp=time.time())
            print(json.dumps(status_data, indent=4, ensure_ascii=False))
            print("------------------\n")
            continue

        # 批量调度命令
        elif user_input.startswith("batch "):
            # 解析批量任务
            parts = user_input.split()[1:]  # 跳过 ‘batch’
            if len(parts) % 3 != 0:
                print("格式错误，请确保每个任务都有对应的机器人类型、目标位置和执行时间")
                # print("示例：batch dog 6_3_G human 4_3_A")
                print("示例：batch dog 6_3_G 2 human 4_3_A 5")
                continue

            batch_tasks = []
            for i in range(0, len(parts), 3):
                skill = parts[i]
                target = parts[i+1]
                duration = int(parts[i+2])
                batch_tasks.append(Task(task_counter, skill, "", target, duration))
                task_counter += 1

            # 确定最佳任务顺序
            print("\n确定最佳任务顺序...")
            best_order, _ = batch_scheduler.find_optimal_schedule(robots, batch_tasks)

            # 执行批量调度
            print("\n执行批量调度...")
            # assignments = batch_scheduler.schedule_batch(batch_tasks)
            assignments = batch_scheduler.schedule_batch(best_order)
            for assignment in assignments:
                print(f"任务 {assignment['task_id']} 分配给机器人 {assignment['robot_id']}")
            continue

        # 单个任务处理
        elif len(user_input.split()) == 3:
            skill, target, duration = user_input.split()
            duration = int(duration)
            task = Task(task_counter, skill, "", target, duration)
            result = batch_scheduler.assign_task(task, now)

            if "error" in result:
                print(f"[!] {result['error']}")
            else:
                print(f"\n系统运行时间: {now:.2f}秒")
                print(f"[OK] 任务分配成功: Robot {result['robot_id']}")
                print(f"    预计开始时间: {result['start_time']:.2f}秒")
                print(f"    预计完成时间: {result['end_time']:.2f}秒")
                task_counter += 1
        else:
            print("格式错误，请输入：<skill> <target_position> <duration>（例如：dog 6_3_G 1.3）")


if __name__ == "__main__":
    # scheduler, _ = batch_scheduling_demo()
    # start_time = int(time.time())
    # 后续调用显式传递 scheduler
    # status = get_robot_status_real_time(scheduler, start_time)
    start_interactive_scheduler()