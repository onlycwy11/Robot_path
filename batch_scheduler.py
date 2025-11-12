# batch_scheduler.py
import json
import time
import math
from typing import List, Dict
from graph import initial_six_graphs
from node import show_path_with_coords, get_coordinates_from_node, get_xyz_from_path_and_time, \
    get_xyz_from_path_and_time_with_elevator_wait
from collections import defaultdict
import copy


# ============================================================
# 全局变量
# ============================================================
current_paths = {}  # 存储每个任务的路径信息
robot_status = {}  # 存储每个机器人的实时状态


class Elevator:
    def __init__(self, eid: int, bldg_num: int, local_id: str, current_floor: int = 1):
        self.id = eid
        self.bldg_num = bldg_num
        self.local_id = local_id
        self.current_floor = current_floor
        # 电梯调度表
        # 开始时间、结束时间、起始楼层、目标楼层、使用电梯的机器人ID
        self.schedule = []  # (start_time, end_time, from_floor, to_floor, robot_id)

    # 电梯预约机制
    def reserve(self, start_time: float, duration: float, from_floor: int, to_floor: int, robot_id: int):
        end_time = start_time + duration
        self.schedule.append((start_time, end_time, from_floor, to_floor, robot_id))
        self.schedule.sort(key=lambda x: x[0])  # 按时间排序
        self.current_floor = to_floor  # 更新电梯当前楼层
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
    def __init__(self, tid: int, skill: str, start: str, target: str):
        self.id = tid
        self.skill = skill
        self.start = start
        self.target = target


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
    global current_paths
    path_results = {}

    # 楼梯路径
    path_stair, cost_stair = stair_graph.dijkstra(start_pos, target_pos)
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
        "1_E1": add_1E1_graph,
        "1_E2": add_1E2_graph,
        "2_E1": add_2E1_graph,
        "2_E2": add_2E2_graph,
        "3_E1": add_3E1_graph,
        "3_E2": add_3E2_graph,
    }

    for eid, g in graph_map.items():
        res = g.dijkstra_extra(start_pos, target_pos)
        if not res or "total_time" not in res or not res["path"]:
            continue
        before = res["segments"]["before"]
        between = res["segments"]["between"]
        after = res["segments"]["after"]
        start_e, end_e = res["E_nodes"]

        if not start_e or not end_e or math.isinf(res["total_time"]):
            continue

        elev = elevators[eid]
        from_floor = int(start_e.split("_")[0])
        # 计算电梯到达起始楼层所需时间
        travel_to_start = abs(elev.current_floor - from_floor) * 1.75
        # 机器人到达电梯前的时刻
        robot_arrival = current_time + before
        # 电梯到达起始楼层的时刻
        elevator_ready = current_time + travel_to_start
        # 机器人和电梯的 ”有效开始时间“ 较晚者
        effective_start = max(robot_arrival, elevator_ready)

        # 初始化等待时间
        wait_time = 0.0

        # 检查电梯调度表中的冲突
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
        elev.reserve(
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
        Task(0, "dog", "", "3_5_A"),
        Task(1, "human", "", "9_2_B"),
        Task(2, "dog", "", "6_3_C"),
        Task(3, "human", "", "3_7_D")
    ]

    # 执行批量调度
    assignments = batch_scheduler.schedule_batch(tasks)

    return scheduler, assignments


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
    print("输入 'exit' 退出系统")
    print("输入 'robot' 查看机器人状态")
    print("输入 'batch' 开始批量调度模式（输入多个任务后一次性调度）")
    print("----------------------------------")

    batch_mode = False
    batch_tasks = []

    while True:
        now = time.time() - batch_scheduler.start_time

        if batch_mode:
            prompt = f"批量模式 (已收集 {len(batch_tasks)} 个任务) > "
        else:
            prompt = "调度系统 > "

        user_input = input(prompt).strip()

        # 退出系统
        if user_input.lower() == "exit":
            print(f"\n系统运行时间: {now:.2f}秒")
            print("退出调度系统")
            break

        # 查看机器人状态
        elif user_inputlower() == "robot":
            print(f"\n系统运行时间: {now:.2f}秒")
            print("\n--- 机器人状态 ---")
            status_data = get_robot_status_real_time(current_timestamp=time.time())
            print(json.dumps(status_data, indent=4, ensure_ascii=False))
            print("------------------\n")
            continue

        # 批量模式切换
        elif user_input.lower() == "batch":
            if batch_mode:
                if batch_tasks:
                    print("\n执行批量调度...")
                    assignments = batch_scheduler.schedule_batch(batch_tasks)
                    for assignment in assignments:
                        print(f"任务 {assignment['task_id']} 分配给机器人 {assignment['robot_id']}")
                    batch_tasks = []
                batch_mode = False
                print("已退出批量模式")
            else:
                batch_mode = True
                print("进入批量模式，请输入多个任务（输入空行结束）：")
            continue

        # 空行退出批量模式
        elif user_input == "" and batch_mode:
            if batch_tasks:
                print("\n执行批量调度...")
                assignments = batch_scheduler.schedule_batch(batch_tasks)
                for assignment in assignments:
                    print(f"任务 {assignment['task_id']} 分配给机器人 {assignment['robot_id']}")
                batch_tasks = []
            batch_mode = False
            continue

        # 批量模式下收集任务
        elif batch_mode:
            if len(user_input.split()) == 2:
                skill, target = user_input.split()
                batch_tasks.append(Task(task_counter, skill, "", target))
                task_counter += 1
                print(f"已添加任务: {skill} -> {target}")
            else:
                print("格式错误，请输入：<skill> <target_position>（例如：dog 6_3_G）")
            continue

        # 单个任务处理
        elif len(user_input.split()) == 2:
            skill, target = user_input.split()
            task = Task(task_counter, skill, "", target)
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
            print("格式错误，请输入：<skill> <target_position>（例如：dog 6_3_G）")


if __name__ == "__main__":
    scheduler, _ = batch_scheduling_demo()
    start_time = int(time.time())
    # 后续调用显式传递 scheduler
    status = get_robot_status_real_time(scheduler, start_time)