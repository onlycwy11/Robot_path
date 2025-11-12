import time
from typing import Dict
from graph import initial_six_graphs  # 假设已有图类，支持 dijkstra 和 dijkstra_extra
from node import show_path_with_coords, get_coordinates_from_node, get_xyz_from_path_and_time, \
    get_xyz_from_path_and_time_with_elevator_wait
import math
import json

# ============================================================
# 全局变量
# ============================================================
current_paths = {}  # 存储每个任务的路径信息
robot_status = {}  # 存储每个机器人的实时状态
start_timestamp = 0


# ============================================================
# Classes
# ============================================================
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
        self.current_floor = to_floor  # 更新电梯的当前楼层
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


# ============================================================
# Elevator Utilities
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


# ============================================================
# Scheduler
# ============================================================
class Scheduler:
    def __init__(self, robots, elevators, stair_graph, elevator_graphs):
        self.robots = robots
        self.elevators = elevators
        self.stair_graph = stair_graph
        self.elevator_graphs = elevator_graphs

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

    # ============================================================
    # 实时获取机器人状态函数
    # ============================================================
    """
    获取当前所有机器人状态，输出前端可用格式：
    posionX/Y/Z 是实时坐标
    """
    """ 获取当前所有机器人状态，输出前端可用格式： 
    { "dataList": 
    [ { "robotId": "0", 
    "robotName": "Dog", 
    "robotType": 1, 
    "status": 0, 
    "posionX": 12.00, 
    "posionY": 3.5, 
    "posionZ": 11.0, 
    "timeStamp": 12321321312 
    }, ... 
    ] 
    } """


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
# Terminal Interface
# ============================================================
def start_terminal_scheduler():
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
    # global scheduler
    global scheduler, start_timestamp  # 声明全局变量

    scheduler = Scheduler(robots, elevators, stair_graph, elevator_graphs)

    start_timestamp = time.time()
    task_counter = 0

    print("=== Multi-Robot Real-Time Scheduler ===")
    print("输入任务格式：<skill> <target_position>，例如：dog 6_3_G, human 4_3_A")
    print("输入 'exit' 退出, 输入 'robot' 查看当前机器人状态")

    # ================= Terminal Loop =================
    while True:
        user_input = input_task("")  # input("New Task> ").strip()
        now = time.time() - start_timestamp  # 确保 now 始终定义

        if user_input.lower() == "exit":
            print(f"\nSchedule running time: {now:.2f}s")
            print("退出调度系统")
            break

        # if user_input.lower() == "robot":
        #     print(f"\nSchedule running time: {now:.2f}s")
        #     print("\n--- 当前机器人状态 ---")
        #     for r in robots:
        #         # 计算 running_time
        #         if r.path_start_time is None or now < r.path_start_time:
        #             r.running_time = 0
        #         elif now < r.path_start_time + r.path_total_time:
        #             r.running_time = now - r.path_start_time
        #         else:
        #             r.running_time = r.path_total_time
        #
        #         # 安全更新 current_position，考虑电梯等待时间
        #         if r.path and len(r.path) > 0:
        #             try:
        #                 r.current_position = get_xyz_from_path_and_time_with_elevator_wait(
        #                     path_list=r.path,
        #                     t=r.running_time,
        #                     wait_time=r.wait_time
        #                 )
        #             except Exception:
        #                 # 任何错误都回退到目标节点坐标
        #                 r.current_position = get_coordinates_from_node(r.position)
        #         else:
        #             r.current_position = get_coordinates_from_node(r.position)
        #
        #         state = "空闲" if now >= r.available_time else f"忙碌({r.available_time - now:.1f}s)"
        #
        #         print(
        #             f"Robot {r.id} ({r.skill}): task_final_pos={r.position}, "
        #             f"current_position={r.current_position}, task_running_time={r.running_time:.2f}s, "
        #             f"available_time={r.available_time:.2f}, 状态={state}"
        #         )
        #     print("----------------------\n")
        #     continue
        if user_input.lower() == "robot":
            print(f"\nSchedule running time: {now:.2f}s")
            print("\n--- 当前机器人状态 ---")

            status_data = get_robot_status_real_time(current_timestamp=time.time())
            # 直接打印整个字典
            # for k, v in status_data.items():
            #     print(f"{k}: {v}")
            # print(status_data)
            print(json.dumps(status_data, indent=4, ensure_ascii=False))

            print("----------------------\n")
            continue

        # 解析任务输入
        if len(user_input.split()) != 2:
            print(f"\nSchedule running time: {now:.2f}s")
            print("格式错误，请输入：<skill> <target_position>")
            continue

        skill, target = user_input.split()
        task = Task(task_counter, skill, "", target)
        result = scheduler.assign_task(task, now)

        if "error" in result:
            print(f"[!] {result['error']}")
        else:
            print(f"\nSchedule running time: {now:.2f}s")
            print(
                f"[OK] 任务分配成功: Robot {result['robot_id']}, Start={result['start_time']:.2f}s, End={result['end_time']:.2f}s")
            task_counter += 1


# ============================================================
# Run
# ============================================================
if __name__ == "__main__":
    start_terminal_scheduler()
