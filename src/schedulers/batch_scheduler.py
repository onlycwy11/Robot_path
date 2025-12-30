# batch_scheduler.py
import json
import time
import math
import threading
from typing import List, Dict, Tuple, Any, DefaultDict
from src.core.graph import initial_six_graphs
from src.core.node import show_path_with_coords, get_coordinates_from_node, \
    get_xyz_from_path_and_time_with_elevator_wait
from collections import defaultdict
import copy
from itertools import permutations, product, groupby
from operator import itemgetter
import paho.mqtt.client as mqtt
import networkx as nx

# ============================================================
# 全局变量
# ============================================================
MIN_CHARGE_THRESHOLD = 40.0
CHARGING_POSITION = "1_1_Left_1"  # 充电点坐标
current_paths = {}  # 存储每个任务的路径信息
robot_status = {}  # 存储每个机器人的实时状态
# 模拟电梯预约
simulated_elevator_reservations = {}  # 格式: {elevator_id: [reservation1, reservation2, ...]}


class Stair:
    """楼梯类，用于检测和管理楼梯超车、会车问题"""
    def __init__(self, stair_id: str, building_num: int, floor_num: int):
        self.id = stair_id  # 格式: "Stair1_1", "Stair2_2"等
        self.building_num = building_num
        self.floor_num = floor_num
        self.usage_schedule = []  # (start_time, end_time, robot_id, direction)

    def reserve(self, start_time: float, duration: float, robot_id: int, task_id: int):
        """预约楼梯使用"""
        end_time = start_time + duration
        self.usage_schedule.append((start_time, end_time, robot_id, task_id))
        self.usage_schedule.sort(key=lambda x: x[0])
        print(f"[Stair {self.id} Reserved] R{robot_id} (Task{task_id}): {start_time:.2f}s - {end_time:.2f}s")

    def check_availability(self, desired_start: float, duration: float) -> Tuple[bool, float, float]:
        """
        检查楼梯可用性
        返回(是否可用, 实际开始时间, 等待时间)
        """
        desired_end = desired_start + duration

        if not self.usage_schedule:
            return True, desired_start, 0.0

        # 检查当前时段是否有冲突
        actual_start = desired_start
        for scheduled_start, scheduled_end, robot_id, task_id in self.usage_schedule:
            if actual_start + duration <= scheduled_start:
                # 找到可用时间段
                break
            elif actual_start < scheduled_end:
                # 有冲突，需要等待
                actual_start = scheduled_end

        wait_time = max(0, actual_start - desired_start)
        return True, actual_start, wait_time

    def copy(self):
        """创建楼梯的深拷贝"""
        new_stair = Stair(self.id, self.building_num, self.floor_num)
        new_stair.usage_schedule = copy.deepcopy(self.usage_schedule)
        return new_stair


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
        根据当前时间和调度表，计算电梯的实际楼层（运行时返回当前调度的目标楼层）
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
                # progress = (current_time - s) / (e - s)  # 0~1
                # current_floor = from_floor + progress * (to_floor - from_floor)
                return to_floor

        # 在所有预约之后
        _, last_e, _, last_to, _ = sorted_sched[-1]
        return last_to

    def check_reserve(
            self, current_time: float, before: float, between: float,
            from_floor: int, elevator_position_at_start: int = 0) -> Tuple:
        """检查是否可以正常预约电梯"""
        # 电梯当前所在楼层
        if not elevator_position_at_start:
            elevator_position_at_start = self.get_current_floor(current_time)
        # 电梯准备就绪所需的移动时间
        travel_to_start = abs(from_floor - elevator_position_at_start) * 1.75

        # 本次电梯预约的开始时间
        elev_start = current_time + before - travel_to_start if before > travel_to_start else current_time
        elev_ready = current_time + travel_to_start

        # 电梯到达起始楼层的时刻
        # elevator_ready = current_time + travel_to_start
        # 机器人和电梯的 ”有效开始时间“ 较晚者，乘坐电梯需保证两者都就位
        # effective_start = max(before + current_time, elevator_ready)

        # 初始化等待时间
        wait_time = 0.0
        delta = 0.0
        new_current_time = current_time

        # 检查电梯调度表中的冲突
        # (开始时间s, 结束时间e, 起始楼层, 目标楼层, 机器人ID)
        for (s, e, _from, _to, _rid) in self.schedule:
            # 如果当前预约时间段与已有预约冲突
            # 无冲突的情况：电梯停止时间早于预约开始时间 或 电梯启动时间晚于预约结束时间
            if not (elev_start + travel_to_start + between <= s or elev_start >= e):
                # 计算需要等待的时间（直到冲突预约结束）
                wait_time = max(wait_time, e - elev_start)
                delta = before - (e - current_time)
                new_current_time = max(new_current_time, e)

        if wait_time:  # 预约有冲突，需至下一个时间段重新预约
            before_new = delta if delta > 0 else 0
            elev_start, elev_end, elev_ready = self.check_reserve(new_current_time, before_new, between, from_floor)
            return elev_start, elev_end, elev_ready

        elev_end = elev_start + wait_time + travel_to_start + between
        return elev_start, elev_end, elev_ready

    def can_reserve_early_move(self, t_now: float, current_floor: int, target_floor: int) -> Tuple[bool, float]:
        """检查是否可以提前调度电梯到目标楼层（不干扰现有预约）"""
        # 找到下一个任务
        next_task = None
        previous_task = None
        for i, task in enumerate(self.schedule):
            if task[0] > t_now:  # 找到第一个开始时间 > t_now 的任务
                next_task = task
                if i > 0:  # 确保不是第一个任务
                    previous_task = self.schedule[i - 1]
                break

        # 计算空闲时段
        free_start = previous_task[1] if previous_task and t_now < previous_task[1] else t_now
        free_end = next_task[0] if next_task else float('inf')  # 无后续任务时无限空闲

        # 估算移动时间（简化：每层楼耗时1秒）
        move_time = abs(target_floor - current_floor) * 1.75

        # 检查是否能在空闲时段完成移动
        if free_start + move_time < free_end:
            return True, move_time
        else:
            # 若无法完成，则等待下一个空闲时间段
            current_floor = self.get_current_floor(next_task[1])
            _, move_time = self.can_reserve_early_move(next_task[1], current_floor, target_floor)
            return False, move_time + next_task[1] - t_now

    # 电梯预约机制
    def reserve(self, start_time: float, end_time: float, from_floor: int, to_floor: int, robot_id: int):
        self.schedule.append((start_time, end_time, from_floor, to_floor, robot_id))
        self.schedule.sort(key=lambda x: x[0])  # 按时间排序
        print(
            f"[Elevator {self.id} Reserved] R{robot_id}: {from_floor}->{to_floor}, {start_time:.2f}s - {end_time:.2f}s")


class Robot:
    def __init__(self, rid: int, skill: str, position: str, enable_mqtt_charge_updates: bool = False):
        self.id = rid
        self.skill = skill
        self.initial_time = 0

        self.charge = 100.0
        self.expected_charge = 100.0
        self.is_charging = False  # 是否在充电

        self.position = position  # 机器人最后一个任务结束时的位置
        self.expected_position = position
        self.available_time = 0.0  # 秒表时间
        self.expected_available_time = 0.0
        self.task_list = {}  # {任务ID: {"time": 完成时间, ...}}
        self.path = []  # 当前任务路径
        self.path1 = []
        self.path2 = []
        self.path_start_time = None  # 当前任务开始时间
        self.path_total_time = 0.0  # 当前任务总耗时
        self.pick_time = 0.0
        self.deliver_time = 0.0
        self.running_time = 0.0  # 当前任务已执行时间
        self.wait_pair_1 = (0.0, 0.0)
        self.wait_pair_2 = (0.0, 0.0)
        self.current_position = get_coordinates_from_node(position)  # 初始xyz坐标

        self.enable_mqtt_charge_updates = enable_mqtt_charge_updates  # 新增：控制是否启用MQTT电量更新
        self.mqtt_client = None  # 初始化为 None，仅在需要时创建

        # 启动 MQTT 监听（自动更新电量）
        # （根据标志决定是否订阅电量主题）
        if self.enable_mqtt_charge_updates:
            self._start_mqtt_listener()

        # 启动后台线程，定时更新机器人状态（实时计算 running_time 和 position）
        self._running = True
        self._status_update_thread = threading.Thread(target=self._auto_update_status)
        self._status_update_thread.daemon = True
        self._status_update_thread.start()

    def set_initial_time(self,start_time):
        self.initial_time = start_time

    def _start_mqtt_listener(self):
        """仅在启用 MQTT 时调用此方法"""
        def on_connect(client, userdata, flags, rc):
            client.subscribe(f"robot/{self.id}/charge")

        def on_message(client, userdata, msg):
            try:
                # 解析JSON格式的电量数据
                payload = json.loads(msg.payload.decode())
                new_charge = float(payload["charge_level"])

                if new_charge < 0:
                    new_charge = 0.0
                elif new_charge > 100.0:
                    new_charge = 100.0

                # 电量不足时，强制返回充电点
                if new_charge < MIN_CHARGE_THRESHOLD and not self.is_charging:
                    self.charge = 0.0
                    self.is_charging = True
                    self.position = CHARGING_POSITION
                    self.current_position = get_coordinates_from_node(CHARGING_POSITION)
                    self.path = []  # 停止当前任务
                    # self.task_list.clear()  # 清空任务列表
                elif self.is_charging:
                    # 充电中，电量只能增加
                    if new_charge > self.charge:
                        self.charge = new_charge
                    # 充满电后恢复
                    if self.charge >= 100.0:
                        self.is_charging = False
                        self.charge = 100.0
                else:
                    self.charge = new_charge  # 正常更新电量
            except Exception as e:
                print(f"Error updating charge: {e}")

        try:
            self.mqtt_client = mqtt.Client()
            self.mqtt_client.on_connect = on_connect
            self.mqtt_client.on_message = on_message
            self.mqtt_client.connect("localhost", 1883)  # 替换为你的 Broker 地址
            self.mqtt_client.loop_start()
        except Exception as e:
            print(f"[WARNING] MQTT 初始化失败，已禁用电量更新: {e}")
            self.enable_mqtt_charge_updates = False  # 回退到禁用状态

    def set_mqtt_charge_updates(self, enable: bool):
        """动态设置是否启用MQTT电量更新"""
        if enable == self.enable_mqtt_charge_updates:
            return  # 无变化

        self.enable_mqtt_charge_updates = enable
        if enable:
            try:
                if not self.mqtt_client:
                    self._start_mqtt_listener()
                else:
                    self.mqtt_client.subscribe(f"robot/{self.id}/charge")
            except Exception as e:
                print(f"[ERROR] 启用 MQTT 失败: {e}")
                self.enable_mqtt_charge_updates = False
        else:
            if self.mqtt_client:
                # 如果禁用，取消订阅
                self.mqtt_client.unsubscribe(f"robot/{self.id}/charge")

    def __del__(self):
        """清理 MQTT 客户端"""
        if self.mqtt_client:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()

    def _auto_update_status(self):
        """后台线程，定时更新机器人状态（running_time 和 position）"""
        while self._running:
            current_time = time.time() - self.initial_time   # 可以用你的秒表时间替代
            self._update_running_status(current_time)
            time.sleep(0.5)  # 每 100ms 更新一次，调整频率

    def _update_running_status(self, current_time):
        """实时更新 running_time 和 current_position"""
        if not self.task_list or self.is_charging:
            return  # 无任务或在充电，不更新

        # 始终更新为最后一个任务的值
        last_task_id, last_task_info = next(reversed(self.task_list.items()))  # 获取最后一个任务
        self.position = last_task_info["path"][-1]  # 更新为最后一个任务的终点
        self.available_time = last_task_info["finish_time"]  # 更新为最后一个任务的结束时间

        # 取最早的任务（假设 task_list 是按时间排序的）
        task_id, task_info = next(iter(self.task_list.items()))
        task_finish_time = task_info["finish_time"]

        # 任务已完成，清理任务
        if current_time >= task_finish_time:
            self.remove_task(task_id)
            self.path = []
            self.path_start_time = None
            self.path_total_time = 0.0
            self.running_time = 0.0
            return

        # 任务刚开始，初始化 path 和时间
        if self.path_start_time is None:
            self.path_start_time = task_info.get("start_time", current_time)
            self.path = task_info.get("path", [])
            self.path1 = task_info.get("path1", [])
            self.path2 = task_info.get("path2", [])
            self.path_total_time = task_finish_time - self.path_start_time
            self.pick_time = task_info.get("pick_time", 0)
            self.deliver_time = task_info.get("deliver_time", 0)
            self.wait_pair_1 = task_info.get("wait_pair_1", 0)
            self.wait_pair_2 = task_info.get("wait_pair_2", 0)

        # 计算已运行时间
        self.running_time = current_time - self.path_start_time
        if self.running_time < 0:
            self.running_time = 0.0
        elif self.running_time > self.path_total_time:
            self.running_time = self.path_total_time

        # 更新实时位置
        if self.path:
            try:
                if self.running_time < self.pick_time:
                    wait_time_1, wait_time_2 = self.wait_pair_1
                    x, y, z = get_xyz_from_path_and_time_with_elevator_wait(
                        path_list=self.path1,
                        t=self.running_time,
                        wait_time_1=wait_time_1,
                        wait_time_2=wait_time_2
                    )
                    self.current_position = (x, y, z)
                else:
                    wait_time_1, wait_time_2 = self.wait_pair_2
                    x, y, z = get_xyz_from_path_and_time_with_elevator_wait(
                        path_list=self.path2,
                        t=self.running_time - self.pick_time,
                        wait_time_1=wait_time_1,
                        wait_time_2=wait_time_2
                    )
                    self.current_position = (x, y, z)
            except Exception:
                pass  # 保持原位置

    def stop(self):
        """停止后台线程和 MQTT 客户端"""
        self._running = False
        if hasattr(self, "mqtt_client"):
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()

    def calculate_electricity_consumption(self, full_path):  # TODO: 电量消耗函数等待完善
        """计算预计电量消耗"""
        return 0.0

    def add_task(
            self, task_id: int, start_time: float, finish_time: float,
            path1, path2, actual_time1: float, actual_time2: float,
            wait_time_1: float, wait_time_2: float, wait_time_3: float, wait_time_4: float, **kwargs
    ):
        full_path = merge_paths(path1, path2)
        """添加任务，可传入任意额外参数"""
        self.task_list[task_id] = {
            "task_id": task_id,
            "start_time": start_time,
            "finish_time": finish_time,
            "path": full_path,
            "path1": path1,
            "path2": path2,
            "pick_time": actual_time1,
            "deliver_time": actual_time2,
            "wait_pair_1": (wait_time_1, wait_time_2),
            "wait_pair_2": (wait_time_3, wait_time_4),
            **kwargs
        }  # **kwargs 接收任意数量的 关键字参数（打包成字典 dict）

    def remove_task(self, task_id: int):
        """删除任务"""
        if task_id in self.task_list:
            return self.task_list.pop(task_id)
        return None

    def get_task(self, task_id: int):
        """按任务ID查找任务"""
        return self.task_list.get(task_id)

    def get_sorted_tasks(self):
        """获取按开始时间排序的任务列表（延迟排序）"""
        sorted_items = sorted(
            self.task_list.items(),
            key=lambda x: x[1]["start_time"]
        )

        self.task_list = dict(sorted_items)


class Task:
    def __init__(self, tid: int, skill: str, start: str, target: str, priority: int = 3,
                 pickup_duration: float = 0.0, deliver_duration: float = 0.0):
        self.id = tid
        self.skill = skill
        self.start = start  # 取药点
        self.target = target  # 送药点
        self.priority = priority
        self.pickup_duration = pickup_duration
        self.deliver_duration = deliver_duration  # 任务持续时间


def init_six_elevators() -> Dict[str, Elevator]:
    elevators = {}
    elevators["1_E1"] = Elevator(1, 1, "E1")
    elevators["1_E2"] = Elevator(2, 1, "E2")
    elevators["2_E1"] = Elevator(3, 2, "E1")
    elevators["2_E2"] = Elevator(4, 2, "E2")
    elevators["3_E1"] = Elevator(5, 3, "E1")
    elevators["3_E2"] = Elevator(6, 3, "E2")
    return elevators


def check_condition(start, target):
    """是否需要两台电梯
        应用场景：跨楼运输，且起点、终点均不在1层
    """
    # 分割字符串
    start_parts = start.split("_")  # [x, y, z]
    target_parts = target.split("_")  # [x, y, z]

    # 提取楼层和楼号
    start_floor = int(start_parts[0])  # x
    start_building = start_parts[1]  # y

    target_floor = int(target_parts[0])  # x
    target_building = target_parts[1]  # y

    # 检查条件
    if (start_building != target_building and  # 不在同一栋楼
            start_floor != 1 and  # start不在1层
            target_floor != 1):  # target不在1层
        return True
    else:
        return False


def extract_building_number(room_node):
    if "_" in room_node:
        parts = room_node.split("_")
        if len(parts) >= 2:
            building_num = parts[1]
        else:
            building_num = "1"
    else:
        building_num = "1"

    return building_num


def create_elevator_access_node(room_node, elevator_marker):
    """
    创建电梯访问节点
    格式：1_楼号_电梯编号
    """
    try:
        # 从房间节点提取楼号
        if "_" in room_node:
            parts = room_node.split("_")
            if len(parts) >= 2:
                building_num = parts[1]
            else:
                building_num = "1"
        else:
            building_num = "1"

        # 从电梯标记提取电梯编号
        if "_" in elevator_marker:
            parts = elevator_marker.split("_")
            if len(parts) == 2:
                elevator_code = parts[1]
            else:
                elevator_code = "E1"
        else:
            # 如果没下划线，假设整个就是电梯编号
            elevator_code = elevator_marker

        return f"1_{building_num}_{elevator_code}"
    except Exception as e:
        print(f"创建电梯节点出错: {e}")
        return "1_1_E1"  # 返回默认值


def merge_paths(path1: list, path2: list) -> list:
    """
    合并两个路径，并去除重复的中间节点
    （path1 的最后一个节点和 path2 的第一个节点必须相同）
    """
    if not path1 or not path2:
        return path1 + path2  # 如果其中一个为空，直接返回另一个

    if path1[-1] != path2[0]:
        raise ValueError("路径无法合并：path1 的最后一个节点和 path2 的第一个节点不匹配！")

    # 合并路径（去掉 path1 的最后一个节点，避免重复）
    merged_path = path1[:-1] + path2
    return merged_path


def has_stairs(path_info: Dict) -> bool:
    if path_info["type"] == "stair":
        return True

    path = path_info["path"]

    for i in range(len(path)):
        if "stair" in path[i]:
            return True

    return False


def simulate_elevator_reserve(best_info, rid: int):
    """模拟电梯预约"""
    global simulated_elevator_reservations  # 声明使用全局变量

    if best_info["type"] == "elevator":
        # 模拟预约电梯 1
        eid_1 = best_info["eid_1"]
        if eid_1:
            from_floor_1 = int(best_info["start_e1"].split("_")[0])
            to_floor_1 = int(best_info["end_e1"].split("_")[0])
            reserve_start_abs = best_info["elev_start_1"]
            reserve_end_abs = best_info["elev_end_1"]

            # 记录模拟预约（而不是调用 elev_1.reserve()）
            if eid_1 not in simulated_elevator_reservations:
                simulated_elevator_reservations[eid_1] = []
            simulated_elevator_reservations[eid_1].append({
                "start_time": reserve_start_abs,
                "end_time": reserve_end_abs,
                "from_floor": from_floor_1,
                "to_floor": to_floor_1,
                "robot_id": rid
            })

        # 模拟预约电梯 2
        eid_2 = best_info["eid_2"]
        if eid_2:
            from_floor_2 = int(best_info["start_e2"].split("_")[0])
            to_floor_2 = int(best_info["end_e2"].split("_")[0])
            reserve_start_abs = best_info["elev_start_2"]
            reserve_end_abs = best_info["elev_end_2"]

            if eid_2 not in simulated_elevator_reservations:
                simulated_elevator_reservations[eid_2] = []
            simulated_elevator_reservations[eid_2].append({
                "start_time": reserve_start_abs,
                "end_time": reserve_end_abs,
                "from_floor": from_floor_2,
                "to_floor": to_floor_2,
                "robot_id": rid
            })


def cancel_elevator_reservation(elevator_id: str, target_start_time: float, target_robot_id: int):
    """
    取消指定电梯的某个预约（根据 start_time 和 robot_id 匹配）

    Args:
        elevator_id (int): 电梯ID
        target_start_time (float): 要取消的预约的开始时间
        target_robot_id (int): 要取消的预约的机器人ID
    """
    global simulated_elevator_reservations

    if elevator_id not in simulated_elevator_reservations:
        print(f"电梯 {elevator_id} 没有预约记录")
        return

    # 获取该电梯的所有预约
    reservations = simulated_elevator_reservations[elevator_id]

    # 遍历预约列表，找到匹配的记录
    for i, reservation in enumerate(reservations):
        if (reservation["start_time"] == target_start_time
                and reservation["robot_id"] == target_robot_id):
            # 删除匹配的预约
            del reservations[i]
            print(f"已取消电梯 {elevator_id} 的预约（start_time={target_start_time}, robot_id={target_robot_id}）")

            # # 如果该电梯的预约列表为空，可选择删除该电梯的条目
            # if not reservations:
            #     del simulated_elevator_reservations[elevator_id]
            #     print(f"电梯 {elevator_id} 的预约列表已清空，已删除该电梯的条目")
            # return

    print(f"未找到匹配的预约（start_time={target_start_time}, robot_id={target_robot_id}）")


# ============================================================
# Path Selection
# ============================================================
def select_best_path_with_elevator(
        task: Task,
        rid: int,
        start_pos: str,
        pickup_pos: str,
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
    tid = task.id

    # 楼梯路径
    # 调用Dijkstra算法计算纯楼梯路径的最短路径和耗时
    if pickup_pos:
        # 取药品
        path_stair_pickup, cost_stair_pickup = stair_graph.dijkstra(start_pos, pickup_pos)
        # 若找到有效路径且代价不是无穷大
        if path_stair_pickup and not math.isinf(cost_stair_pickup):
            path_results["stair_1"] = {
                "path": path_stair_pickup,
                "start_time": current_time,
                "actual_time": cost_stair_pickup,
                "part_time_2": cost_stair_pickup,
                "wait_time_2": 0.0,
                "before": 0.0,
                "between": 0.0,
                "after": 0.0,
                "eid_2": None,
                "type": "stair",
                "status": 0,
            }

        # 送药品
        path_stair_delivery, cost_stair_delivery = stair_graph.dijkstra(pickup_pos, target_pos)
    else:
        # 送药品
        path_stair_delivery, cost_stair_delivery = stair_graph.dijkstra(start_pos, target_pos)

    # 若找到有效路径且代价不是无穷大
    if path_stair_delivery and not math.isinf(cost_stair_delivery):
        path_results["stair_2"] = {
            "path": path_stair_delivery,
            "start_time": current_time,
            "actual_time": cost_stair_delivery,
            "part_time_2": cost_stair_delivery,
            "wait_time_2": 0.0,
            "before": 0.0,
            "between": 0.0,
            "after": 0.0,
            "eid": None,
            "type": "stair",
            "status": 1
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

    start = start_pos
    if pickup_pos:
        segment_index = 0
        target = pickup_pos
    else:
        segment_index = 1
        target = target_pos

    actual_time = 0.0
    while segment_index <= 1:  # 0 = 取药品，1 = 送药品
        for eid, g in graph_map.items():  # 遍历6部电梯
            # 若存在需乘坐两次电梯的跨楼运输
            if check_condition(start, target):
                transit = create_elevator_access_node(start, eid)  # 两次搭乘电梯的中转位置
                building_num = extract_building_number(start)
                if eid.split("_")[0] == building_num:  # 排除无法使用的电梯
                    res_1 = g.dijkstra_extra(start, transit)
                    if not res_1 or "total_time" not in res_1 or not res_1["path"]:
                        continue

                    for eid_1, g_1 in graph_map.items():
                        building_num = extract_building_number(target)
                        if eid_1.split("_")[0] == building_num:  # 排除无法使用的电梯
                            res_2 = g_1.dijkstra_extra(transit, target)
                            if not res_2 or "total_time" not in res_2 or not res_2["path"]:
                                continue

                            before = res_1["segments"]["before"]  # 走到电梯的时间
                            between_1 = res_1["segments"]["between"]  # 第一部电梯运行时间
                            transfer = res_2["segments"]["before"]  # 从第一部电梯到第二部电梯的时间
                            between = res_2["segments"]["between"]  # 第二部电梯运行时间
                            after = res_2["segments"]["after"]  # 出电梯到目标的时间
                            start_e1, end_e1 = res_1["E_nodes"]  # 第一部电梯起点和终点
                            start_e2, end_e2 = res_2["E_nodes"]  # 第二部电梯起点和终点
                            path = merge_paths(res_1["path"], res_2["path"])

                            if not start_e1 or not end_e1 or math.isinf(res_1["total_time"]) or \
                                    not start_e2 or not end_e2 or math.isinf(res_2["total_time"]):
                                continue  # 如果电梯结点无效或代价无穷，跳过

                            elev_1 = elevators[eid]
                            elev_2 = elevators[eid_1]
                            from_floor_1 = int(start_e1.split("_")[0])  # 从输入格式中提取起始楼层
                            end_floor_1 = int(end_e1.split("_")[0])
                            from_floor_2 = int(start_e2.split("_")[0])  # 从输入格式中提取起始楼层
                            end_floor_2 = int(end_e2.split("_")[0])

                            # 检查第一部电梯可预约的时间段
                            elev_start_1, elev_end_1, elev_ready_1 = elev_1.check_reserve(current_time, before,
                                                                                          between_1,
                                                                                          from_floor_1)
                            wait_time_1 = max(elev_ready_1 - current_time - before, 0)
                            part_time_1 = elev_end_1 - current_time

                            # 检查第二部电梯可预约的时间段
                            elev_start_2, elev_end_2, elev_ready_2 = elev_2.check_reserve(elev_end_1, transfer, between,
                                                                                          from_floor_2)

                            wait_time_2 = max(elev_ready_2 - elev_end_1 - transfer, 0)
                            part_time_2 = elev_end_2 + after - elev_end_1
                            actual_time = elev_end_2 + after - current_time

                            unique_eid_key = f"{eid}|{eid_1}"
                            path_results[unique_eid_key] = {
                                "path": path,  # 总路径
                                "path_1": res_1["path"],
                                "path_2": res_2["path"],
                                "start_time": current_time,  # 任务起始时间
                                "actual_time": actual_time,  # 总耗时
                                "part_time_1": part_time_1,
                                "part_time_2": part_time_2,
                                "wait_time_1": wait_time_1,  # 分段等待时间
                                "wait_time_2": wait_time_2,
                                "before": before,  # 各段耗时
                                "between_1": between_1,  # 第一部电梯内耗时
                                "transfer": transfer,
                                "between": between,  # 第二部电梯内耗时
                                "after": after,
                                "start_e1": start_e1,  # 第一部电梯起点和终点
                                "end_e1": end_e1,
                                "start_e2": start_e2,  # 第二部电梯起点和终点
                                "end_e2": end_e2,
                                "eid_1": eid,  # 第一部电梯
                                "eid_2": eid_1,  # 第二部电梯
                                "type": "elevator",
                                "from_floor_1": from_floor_1,  # 楼层信息
                                "end_floor_1": end_floor_1,
                                "from_floor_2": from_floor_2,
                                "end_floor_2": end_floor_2,
                                "elev_start_1": elev_start_1,  # 电梯调度起始时间
                                "elev_end_1": elev_end_1,  # 电梯调度结束时间
                                "elev_start_2": elev_start_2,  # 电梯调度起始时间
                                "elev_end_2": elev_end_2,  # 电梯调度结束时间
                                "status": segment_index
                            }

            # 楼内运输，或跨楼运输但仅使用一次电梯
            building_num1 = extract_building_number(start)
            building_num2 = extract_building_number(target)
            if eid.split("_")[0] == building_num1 or eid.split("_")[0] == building_num2:  # 排除无法使用的电梯
                res = g.dijkstra_extra(start, target)
                # 增强型的Dijkstra，返回包含电梯结点信息的详细结果
                # print(res)

                # 若无结果或无路径，跳过
                if not res or "total_time" not in res or not res["path"]:
                    continue

                before = res["segments"]["before"]  # 走到电梯的时间
                between = res["segments"]["between"]  # 电梯运行时间
                after = res["segments"]["after"]  # 出电梯到目标的时间
                start_e2, end_e2 = res["E_nodes"]  # 电梯起点和终点

                if not start_e2 or not end_e2 or math.isinf(res["total_time"]):
                    continue  # 如果电梯结点无效或代价无穷，跳过

                elev_2 = elevators[eid]
                from_floor_2 = int(start_e2.split("_")[0])  # 从输入格式中提取起始楼层
                end_floor_2 = int(end_e2.split("_")[0])

                # 检查电梯可预约的时间段
                elev_start_2, elev_end_2, elev_ready_2 = elev_2.check_reserve(current_time, before, between,
                                                                              from_floor_2)

                wait_time_2 = max(elev_ready_2 - current_time - before, 0)
                actual_time = elev_end_2 + after - current_time

                path_results[eid] = {
                    "path": res["path"],  # 总路径
                    "start_time": current_time,  # 任务起始时间
                    "actual_time": actual_time,  # 总耗时
                    "part_time_1": 0,
                    "part_time_2": actual_time,
                    "wait_time_1": 0,  # 分段等待时间
                    "wait_time_2": wait_time_2,
                    "before": before,  # 各段耗时
                    "between_1": 0,
                    "transfer": 0,
                    "between": between,
                    "after": after,
                    "start_e1": None,  # 第一部电梯起点和终点
                    "end_e1": None,
                    "start_e2": start_e2,  # 第二部电梯起点和终点
                    "end_e2": end_e2,
                    "eid_1": None,  # 第一部电梯
                    "eid_2": eid,  # 第二部电梯
                    "type": "elevator",
                    "from_floor_1": 0,  # 楼层信息
                    "end_floor_1": 0,
                    "from_floor_2": from_floor_2,
                    "end_floor_2": end_floor_2,
                    "elev_start_1": 0,  # 电梯调度起始时间
                    "elev_end_1": 0,  # 电梯调度结束时间
                    "elev_start_2": elev_start_2,  # 电梯调度起始时间
                    "elev_end_2": elev_end_2,  # 电梯调度结束时间
                    "status": segment_index,
                    "elevator_stair": check_condition(start, target)
                }

        if actual_time:
            current_time = actual_time + current_time
        segment_index = segment_index + 1
        start = pickup_pos
        target = target_pos

    if not path_results:
        print(f"[!] Task {tid} failed: No valid path found from {start_pos} to {target_pos}")
        return None, None, None

    # 从字典 path_results 中找出 actual_time 最小的键（key），即找到最优路径对应的键
    # best_key = min(path_results.keys(), key=lambda k: path_results[k]["actual_time"])
    # best_info = path_results[best_key]
    # real_path = show_path_with_coords(best_info["path"])

    # 1. 筛选取药品的路径（status=0）
    pick_paths = {k: v for k, v in path_results.items() if v["status"] == 0}
    if pick_paths:
        best_pick_key = min(pick_paths.keys(), key=lambda k: pick_paths[k]["actual_time"])
        best_pick_info = path_results[best_pick_key]
        pick_path = show_path_with_coords(best_pick_info["path"])
    else:
        print("无取药品路径")
        best_pick_key = None
        best_pick_info = None
        pick_path = None

    # 2. 筛选送药品的路径（status=1）
    deliver_paths = {k: v for k, v in path_results.items() if v["status"] == 1}
    if deliver_paths:
        best_deliver_key = min(deliver_paths.keys(), key=lambda k: deliver_paths[k]["actual_time"])
        best_deliver_info = path_results[best_deliver_key]
        deliver_path = show_path_with_coords(best_deliver_info["path"])
    else:
        print("无送药品路径")
        best_deliver_key = None
        best_deliver_info = None
        deliver_path = None

    if best_pick_info and best_deliver_info:
        current_paths[tid] = {
            "pick_route": best_pick_key,
            "pick_path": best_pick_info["path"],
            "pick_real_path": pick_path,
            "pick_total_time": best_pick_info['actual_time'],
            "pick_wait_time": best_pick_info['wait_time_1'] + best_pick_info[
                'wait_time_2'] if 'wait_time_1' in best_pick_info else best_pick_info['wait_time_2'],
            "deliver_route": best_deliver_key,
            "deliver_path": best_deliver_info["path"],
            "deliver_real_path": deliver_path,
            "deliver_total_time": best_deliver_info['actual_time'],
            "deliver_wait_time": best_deliver_info['wait_time_1'] + best_deliver_info['wait_time_2'] if
            best_deliver_info[
                'wait_time_1'] else best_deliver_info['wait_time_2']
        }

        # print(
        #     f"\nTask {tid} (pick up) selected route: {best_pick_key}, Total time: {best_pick_info['actual_time']:.2f}s "
        #     f"(wait {current_paths[tid]['pick_wait_time']:.2f}s)")
        # print(f"Path: {best_pick_info['path']}\n")
        # print(f"Real Path: {pick_path}\n")
        #
        # print(
        #     f"\nTask {tid} (deliver) selected route: {best_deliver_key}, Total time: {best_deliver_info['actual_time']:.2f}s "
        #     f"(wait {current_paths[tid]['deliver_wait_time']:.2f}s)")
        # print(f"Path: {best_deliver_info['path']}\n")
        # print(f"Real Path: {deliver_path}\n")

    # # 为最佳路径预约电梯（模拟版）
    # simulate_elevator_reserve(best_pick_info, rid)
    # simulate_elevator_reserve(best_deliver_info, rid)

    return best_pick_info, best_deliver_info, path_results


def generate_constrained_permutations(tasks: List[Task]):
    # 1. 按 priority 和 skill 排序
    sorted_tasks = sorted(
        tasks,
        key=lambda x: (x.priority, 0 if x.skill == "dog" else 1)
    )

    # 2. 按 (priority, skill) 分组
    grouped_tasks: DefaultDict[Tuple[int, str], list] = defaultdict(list)
    for task in sorted_tasks:
        grouped_tasks[task.priority, task.skill].append(task)

    # 3. 对每个组生成排列（如果组内顺序可以交换）
    permuted_groups = []
    for group in grouped_tasks.values():
        if len(group) > 1:
            permuted_groups.append(permutations(group))
        else:
            permuted_groups.append([tuple(group)])  # 单个任务

    # 4. 计算所有可能的组合（笛卡尔积）
    for group_perms in product(*permuted_groups):
        # 展平所有排列
        flattened = []
        for perm in group_perms:
            flattened.extend(perm)
        yield flattened


class BatchScheduler:
    def __init__(self, robots, elevators, stair_graph, elevator_graphs):
        self.robots = robots
        self.elevators = elevators
        self.stair_graph = stair_graph
        self.elevator_graphs = elevator_graphs
        self.robot_plans = {}  # 存储机器人的任务计划
        self.elevator_schedules = defaultdict(list)  # 电梯调度表
        self.simulated_stair_reservations = defaultdict(list)
        self.start_time = time.time()

        self._set_robot_start()

    def _set_robot_start(self):
        for robot in self.robots:
            robot.set_initial_time(self.start_time)

    def schedule_batch(self, tasks: List[Task]):
        """
        批量调度主函数
        """
        current_time = time.time() - self.start_time
        min_tasks_total_time = float('inf')  # 最小总耗时，初始为无穷大
        # initial_assignments = None  # 最优任务分配方案
        best_final_assignments = None

        print(f"\n=== 开始批量调度 {len(tasks)} 个任务 ===")

        # 遍历所有可能的组内排列组合
        # - 对每个技能组的任务列表，生成所有可能的排列（permutations）
        # - product(*[...]) 用于组合多个组的排列，形成所有可能的跨组排列
        i = 0
        for perm_tasks in generate_constrained_permutations(tasks):
            # 1. 执行初始任务分配 - 为每个任务找到技能匹配且时间最优的机器人
            i += 1
            print(f"正在尝试第 {i} 次初始任务分配...")
            # best_assignments, tasks_total_time = self._initial_assignment(perm_tasks, current_time)
            #
            # # 1.1 记录最优解
            # if tasks_total_time < min_tasks_total_time:
            #     min_tasks_total_time = tasks_total_time
            #     initial_assignments = best_assignments
            #
            # print("\n")

            # 1. 初始任务分配
            initial_assignments, _ = self._initial_assignment(perm_tasks, current_time)

            # 2. 检测电梯冲突
            max_iterations = 10  # 防止无限循环
            iteration = 0
            final_assignments = initial_assignments.copy()

            while iteration < max_iterations:
                conflicts = self._detect_elevator_conflicts(final_assignments)

                if not conflicts:
                    print("未检测到电梯冲突")
                    break

                print(f"检测到 {len(conflicts)} 个电梯冲突")
                # 3. 解决冲突（这里可以切换策略1或策略2）
                final_assignments = self._resolve_conflicts(final_assignments, conflicts, strategy="strategy2")
                iteration += 1

            if iteration == max_iterations:
                print(f"[警告] 达到最大迭代次数 {max_iterations}，但仍存在冲突")

            # 4. 楼梯“超车”“会车”问题
            final_assignments = self._resolve_stair_conflicts(final_assignments)

            # 5. 记录最优解
            tasks_total_time = self.calculate_tasks_total_time(final_assignments)
            if tasks_total_time < min_tasks_total_time:
                min_tasks_total_time = tasks_total_time
                best_final_assignments = final_assignments

            print("\n")

        # 6. 执行调度
        self._execute_assignments(best_final_assignments, current_time)

        # for robot in self.robots:
        #     print(robot.task_list)

        return best_final_assignments

    def _initial_assignment(self, tasks: List[Task], current_time: float) -> Tuple[List[dict], float]:
        """
        初始任务分配：为每个任务选择最快到达的机器人
        """
        assignments = []
        tasks_total_time = 0.0

        for robot in self.robots:
            robot.expected_position = robot.position
            robot.expected_available_time = robot.available_time
            robot.expected_charge = robot.charge

        for task in tasks:
            best_robot = None
            best_total_time = float('inf')
            best_path_results = None
            early_start = float('inf')
            pick_best_path_info = None
            deliver_best_path_info = None

            # 找到技能匹配的机器人
            feasible_robots = self.find_feasible_robots(task)

            if not feasible_robots:
                print(f"[警告] 任务 {task.id} 没有匹配技能的机器人")
                continue

            for robot in feasible_robots:
                # print(f"\n可用机器人 {robot.id}")
                # print("新一轮预计时间：")
                # print(robot.id, robot.expected_available_time)
                # 计算机器人到任务目标的最优路径
                pick_path_info, deliver_path_info, path_results = select_best_path_with_elevator(
                    task=task,
                    rid=robot.id,
                    start_pos=robot.expected_position,
                    pickup_pos=task.start,
                    target_pos=task.target,
                    stair_graph=self.stair_graph,
                    add_1E1_graph=self.elevator_graphs["1_E1"],
                    add_1E2_graph=self.elevator_graphs["1_E2"],
                    add_2E1_graph=self.elevator_graphs["2_E1"],
                    add_2E2_graph=self.elevator_graphs["2_E2"],
                    add_3E1_graph=self.elevator_graphs["3_E1"],
                    add_3E2_graph=self.elevator_graphs["3_E2"],
                    elevators=self.elevators,
                    current_time=max(current_time, robot.expected_available_time)
                )

                if not pick_path_info or not deliver_path_info:
                    continue

                pick_total_time = pick_path_info["actual_time"]
                deliver_total_time = deliver_path_info["actual_time"]
                start_time = pick_path_info["start_time"]
                current_total_time = pick_total_time + deliver_total_time
                # print(current_total_time, best_total_time)
                # print(start_time, early_start)
                if current_total_time < best_total_time:
                    best_robot = robot
                    pick_best_path_info = copy.deepcopy(pick_path_info)
                    deliver_best_path_info = copy.deepcopy(deliver_path_info)
                    best_total_time = pick_total_time + deliver_total_time
                    best_path_results = copy.deepcopy(path_results)
                    early_start = start_time
                elif current_total_time == best_total_time and start_time < early_start:
                    best_robot = robot
                    pick_best_path_info = copy.deepcopy(pick_path_info)
                    deliver_best_path_info = copy.deepcopy(deliver_path_info)
                    best_total_time = pick_total_time + deliver_total_time
                    best_path_results = copy.deepcopy(path_results)
                    early_start = start_time

            if best_robot and pick_best_path_info and deliver_best_path_info:
                selected = []
                for key, path_result in best_path_results.items():
                    if pick_best_path_info == path_result:
                        selected.append(key)
                        # print("已正确标记最佳取药品路线！")
                    elif deliver_best_path_info == path_result:
                        selected.append(key)
                        # print("已正确标记最佳送药品路线！")

                assignments.append({
                    "task": task,
                    "robot_id": best_robot.id,
                    "pick_path_info": copy.deepcopy(pick_best_path_info),
                    "deliver_path_info": copy.deepcopy(deliver_best_path_info),
                    "path_results": copy.deepcopy(best_path_results),
                    "start_time": max(current_time, best_robot.expected_available_time),
                    "end_time": max(current_time, best_robot.expected_available_time) + best_total_time,
                    "selected": selected
                })

                tasks_total_time += best_total_time
                print(f"任务 {task.id} 分配给机器人 {best_robot.id}, 预计时间: {best_total_time:.2f}s")

                # 为最佳路径预约电梯（模拟版）
                if pick_best_path_info["type"] == "elevator":
                    simulate_elevator_reserve(pick_best_path_info, best_robot.id)
                if deliver_best_path_info["type"] == "elevator":
                    simulate_elevator_reserve(deliver_best_path_info, best_robot.id)

                full_path = merge_paths(
                    pick_best_path_info["path"], deliver_best_path_info["path"])

                # 更新机器人状态
                for robot in self.robots:
                    if robot.id == best_robot.id:
                        robot.expected_position = task.target
                        # best_robot.path = full_path
                        path_start_time = max(current_time, best_robot.available_time)
                        robot.expected_available_time = path_start_time + best_total_time
                        robot.expected_charge = robot.expected_charge - robot.calculate_electricity_consumption(
                            full_path)
                        # print("预计时间：")
                        # print(robot.id, robot.expected_available_time)

        return assignments, tasks_total_time

    def _detect_elevator_conflicts(self, assignments: List[dict]) -> List[dict]:
        """
        检测电梯使用冲突
        """
        conflicts = []
        elevator_usage = defaultdict(list)

        path_info_map = {
            0: "pick_path_info",
            1: "deliver_path_info"
        }

        # 收集所有电梯使用信息
        for assignment in assignments:
            for i, path_info_str in path_info_map.items():
                path_info = assignment[path_info_str]
                if path_info["type"] == "elevator":
                    elevator_id_1 = path_info["eid_1"]
                    if elevator_id_1:
                        elevator_id_1 = path_info["eid_1"]
                        start_time_1 = path_info["elev_start_1"]
                        end_time_1 = path_info["elev_end_1"]

                        elevator_usage[elevator_id_1].append({
                            "assignment": assignment,
                            "status": i,
                            "part": 1,  # 表示跨楼运输中的第一段
                            "time_window": (start_time_1, end_time_1),
                            "robot_id": assignment["robot_id"]
                        })

                    elevator_id_2 = path_info["eid_2"]
                    start_time_2 = path_info["elev_start_2"]
                    end_time_2 = path_info["elev_end_2"]

                    elevator_usage[elevator_id_2].append({
                        "assignment": assignment,
                        "status": i,
                        "part": 2,  # 表示跨楼运输中的第二段 或 楼内运输
                        "time_window": (start_time_2, end_time_2),
                        "robot_id": assignment["robot_id"]
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

                    # 无冲突场景：
                    # 1. 下一个时间窗 起始时间晚于 当前时间窗 结束时间
                    # 2. 下一个时间窗 结束时间早于 当前时间窗 起始时间 （已排序，实际不会发生）
                    if not (end1 <= start2 or end2 <= start1):
                        conflicts.append({
                            "elevator_id": elevator_id,
                            "usage1": usage1,
                            "usage2": usage2,
                            "overlap_time": min(end1, end2) - max(start1, start2),  # 重叠时间
                            "start_time": min(start1, start2)
                        })

        conflicts_sorted = sorted(conflicts, key=lambda x: x["start_time"])

        return conflicts_sorted

    def _resolve_conflicts(self, assignments: List[dict], conflicts: List[dict], strategy: str = "strategy2") -> List[
        dict]:
        """
        解决电梯冲突
        """
        # 创建可修改的副本
        resolved_assignments = copy.deepcopy(assignments)
        robot_to_assignment = {a["robot_id"]: a for a in resolved_assignments}

        path_info_map = {
            0: "pick_path_info",
            1: "deliver_path_info"
        }

        change_to_others = set()

        # 按电梯分组冲突
        get_elevator_id = itemgetter("elevator_id")
        conflicts_sorted = sorted(conflicts, key=get_elevator_id)
        for elevator_id, conflicts_in_elevator in groupby(conflicts_sorted, key=get_elevator_id):
            # 同一个电梯前的冲突
            for conflict in conflicts_in_elevator:
                usage1 = conflict["usage1"]
                usage2 = conflict["usage2"]

                if strategy == "strategy1":
                    # 策略1: 最大化电梯利用率
                    winner, loser, best_alternative_route = self._strategy1_priority(usage1, usage2, elevator_id)
                else:
                    # 策略2: 最小化代价增加
                    winner, loser, best_alternative_route = self._strategy2_priority(usage1, usage2, elevator_id)

                winner_id = winner["robot_id"]
                loser_id = loser["robot_id"]

                if winner_id in change_to_others or loser_id in change_to_others:
                    continue

                new_elevator_id = best_alternative_route.get("elevator_id")
                if not new_elevator_id:
                    new_elevator_id = "楼梯路径"
                elif new_elevator_id and new_elevator_id == elevator_id:
                    new_elevator_id = "等待电梯空闲"
                print(
                    f"电梯 {elevator_id} 冲突: 机器人 {winner_id} 获胜, 机器人 {loser_id} 路径需要调整为{new_elevator_id}")

                # new_loser_assignment = self._handle_loser_robot(loser, elevator_id, resolved_assignments)

                # # 更新分配
                # resolved_assignments.remove(loser)
                # resolved_assignments.append(new_loser_assignment)

                if loser_id in robot_to_assignment:
                    assignment = robot_to_assignment[loser_id]
                    status = best_alternative_route["status"]
                    assignment[path_info_map[status]] = best_alternative_route["path_info"]
                    change_to_others.add(loser_id)

                    selected = assignment["selected"]
                    for key, path_result in assignment["path_results"].items():
                        if best_alternative_route["path_info"] == path_result:
                            selected.append(key)
                            assignment["selected"] = selected
                            # print("已正确标记新的最佳路线！")

        global simulated_elevator_reservations
        simulated_elevator_reservations = {}  # 重置为空字典

        for assignment in resolved_assignments:
            pick_path_info = assignment["pick_path_info"]
            deliver_path_info = assignment["deliver_path_info"]

            if "part_time_1" in pick_path_info:
                pick_path_info["actual_time"] = pick_path_info["part_time_1"] + pick_path_info["part_time_2"]
            else:
                pick_path_info["actual_time"] = pick_path_info["part_time_2"]

            if "part_time_1" in deliver_path_info:
                deliver_path_info["actual_time"] = deliver_path_info["part_time_1"] + deliver_path_info["part_time_2"]
            else:
                deliver_path_info["actual_time"] = deliver_path_info["part_time_2"]

            assignment["end_time"] = assignment["start_time"] + pick_path_info[
                "actual_time"] + deliver_path_info["actual_time"]

            # 更新电梯预约表
            if pick_path_info["type"] == "elevator":
                simulate_elevator_reserve(pick_path_info, assignment["robot_id"])
            if deliver_path_info["type"] == "elevator":
                simulate_elevator_reserve(deliver_path_info, assignment["robot_id"])

        return resolved_assignments

    def _strategy1_priority(self, usage1, usage2, elevator_id):
        """
        策略1: 最大化电梯利用率
        """
        # 这里实现策略1的逻辑
        # 暂时返回assignment1为获胜者
        assignment1 = usage1["assignment"]
        status1 = usage1["status"]
        part1 = usage1["part"]
        start1, end1 = usage1["time_window"]
        assignment2 = usage2["assignment"]
        status2 = usage2["status"]
        part2 = usage2["part"]
        start2, end2 = usage2["time_window"]

        best_alternative_route1 = None

        return assignment1, assignment2, best_alternative_route1

    def _strategy2_priority(self, usage1, usage2, elevator_id):
        """
        策略2: 最小化代价增加
        """
        assignment1 = usage1["assignment"]
        status1 = usage1["status"]
        part1 = usage1["part"]
        start1, end1 = usage1["time_window"]
        robot_id1 = usage1["robot_id"]
        assignment2 = usage2["assignment"]
        status2 = usage2["status"]
        part2 = usage2["part"]
        start2, end2 = usage2["time_window"]
        robot_id2 = usage2["robot_id"]

        path_info_map = {
            0: "pick_path_info",
            1: "deliver_path_info"
        }

        path_info_key_1 = path_info_map[status1]
        path_info_key_2 = path_info_map[status2]
        part_time_key_1 = f"part_time_{part1}"
        part_time_key_2 = f"part_time_{part2}"

        current_cost1 = assignment1[path_info_key_1][part_time_key_1]
        current_cost2 = assignment2[path_info_key_2][part_time_key_2]

        penalty1, best_alternative_route1 = self.calculate_penalty_if_lose(assignment1, status1, part1, robot_id2,
                                                                           start2, elevator_id)
        penalty2, best_alternative_route2 = self.calculate_penalty_if_lose(assignment2, status2, part2, robot_id1,
                                                                           start1, elevator_id)

        print(f"策略2比较:")
        print(f"  机器人{assignment1['robot_id']} 当前:{current_cost1:.2f}s 惩罚增量:{penalty1:.2f}s")
        print(f"  机器人{assignment2['robot_id']} 当前:{current_cost2:.2f}s 惩罚增量:{penalty2:.2f}s")

        # 若存在优先级差异，优先级低(数值大)的作为loser
        if assignment1["task"].priority > assignment2["task"].priority:
            print(f"  机器人{assignment2['robot_id']} 作为winner，机器人{assignment1['robot_id']} 作为loser")
            return assignment2, assignment1, best_alternative_route1
        elif assignment1["task"].priority < assignment2["task"].priority:
            print(f"  机器人{assignment1['robot_id']} 作为winner，机器人{assignment2['robot_id']} 作为loser")
            return assignment1, assignment2, best_alternative_route2

        # 选择代价增量小的作为loser
        # 若代价增量相等，任务id大的作为loser
        if penalty1 < penalty2 or (penalty1 == penalty2 and assignment1["task"].id > assignment2["task"].id):
            print(f"  机器人{assignment2['robot_id']} 作为winner，机器人{assignment1['robot_id']} 作为loser")
            return assignment2, assignment1, best_alternative_route1
        else:
            print(f"  机器人{assignment1['robot_id']} 作为winner，机器人{assignment2['robot_id']} 作为loser")
            return assignment1, assignment2, best_alternative_route2

    # 策略2相关
    def calculate_penalty_if_lose(self, assignment, status, part, robot_id2, start2, elevator_id):
        """
        计算机器人竞争失败时的代价增加
        返回：代价增量（秒）
        """
        path_info_map = {
            0: "pick_path_info",
            1: "deliver_path_info"
        }

        # part_time_map = {
        #     1: "part_time_1",
        #     2: "part_time_2"
        # }

        path_info = path_info_map[status]
        part_time_info = f"part_time_{part}"

        # 1. 获取当前使用电梯的代价
        current_cost = assignment[path_info][part_time_info]

        # 2. 查找不使用指定电梯的替代路线
        alternative_routes = self.find_alternative_routes(
            assignment, status, part, current_cost, robot_id2, start2, elevator_id)

        if not alternative_routes:
            # 如果没有替代路线，返回一个很大的代价
            print("无替代路线！")
            return float('inf'), None

        # 3. 找到最快替代路线的代价（替代路径代价不能小于原代价）
        # best_alternative_cost = min(route["actual_time"] for route in alternative_routes)

        best_alternative_route = None
        best_alternative_delta = float('inf')
        for alternative_route in alternative_routes:
            if alternative_route["type"] == "elevator_stair":
                original_actual_time = alternative_route["original_actual_time"]
                new_time = alternative_route["part_time_2"]
                delta = new_time - original_actual_time
            elif alternative_route["type"] == "stair":
                original_actual_time = alternative_route["original_actual_time"]
                new_time = alternative_route["part_time_2"]
                delta = new_time - original_actual_time
            elif part == 1:
                original_actual_time = alternative_route["original_actual_time"]
                new_time_1 = alternative_route["part_time_1"]
                new_time_2 = alternative_route["part_time_2"]
                delta = new_time_1 + new_time_2 - original_actual_time
            else:
                delta = alternative_route[part_time_info] - current_cost

            if delta < best_alternative_delta:
                best_alternative_delta = delta
                best_alternative_route = alternative_route

        # best_alternative_route = min(
        #     alternative_routes,
        #     key=lambda route: route[part_time_info]  # 按 part_time 比较
        # )
        # best_alternative_cost = best_alternative_route[part_time_info]

        # 4. 计算代价增量
        penalty = best_alternative_delta

        # 已确保 penalty 非负
        return penalty, best_alternative_route

    def find_alternative_routes(
            self, assignment, status, part, current_cost, robot_id2, start2, forbidden_elevator_id):
        """
        查找不使用指定电梯的替代路线
        注意：替代路线的代价不能小于原路线，否则会引入死锁
        返回：替代路线列表，每条路线包含路径信息和时间
        """
        alternative_routes = []

        path_info_map = {
            0: "pick_path_info",
            1: "deliver_path_info"
        }

        # 从assignment中获取已计算的所有路径
        path_results = assignment["path_results"]
        # print(path_results)
        path_info = path_info_map[status]

        # 1. 楼梯路径（总是可用的替代方案）
        for stair_key in ["stair_1", "stair_2"]:
            if stair_key in path_results:
                stair_route = path_results[stair_key]  # 获取对应的值
                eid = None
                # 取药品/送药品 过程冲突
                original_actual_time = assignment[path_info]["actual_time"]
                if stair_route.get("type") == "stair" and stair_route.get("status") == status:
                    # 若存在跨楼运输
                    # if "part_time_1" in assignment[path_info] and "path_1" in assignment[path_info]:
                    #     start_position = assignment[path_info]["path_1"][0]
                    #     end_position = assignment[path_info]["path_2"][-1]
                    #     current_time = assignment[path_info]["start_time"]
                    #
                    #     if part == 1:  # 第一部电梯冲突，则只使用第二部电梯
                    #         eid = assignment[path_info]["eid_2"]
                    #     elif part == 2:  # 第二部电梯冲突，则只使用第一部电梯
                    #         eid = assignment[path_info]["eid_1"]
                    #
                    #     res = graph_map[eid].dijkstra_extra(start_position, end_position)
                    #     before = res["segments"]["before"]  # 走到电梯的时间
                    #     between = res["segments"]["between"]  # 电梯运行时间
                    #     after = res["segments"]["after"]  # 出电梯到目标的时间
                    #     start_e, end_e = res["E_nodes"]  # 电梯起点和终点
                    #
                    #     if not start_e or not end_e or math.isinf(res["total_time"]):
                    #         print("混合路径走楼梯更快！")  # 如果电梯结点无效或代价无穷，跳过
                    #     else:
                    #         elev = self.elevators[eid]
                    #         from_floor_2 = int(start_e.split("_")[0])  # 从输入格式中提取起始楼层
                    #         end_floor_2 = int(end_e.split("_")[0])
                    #
                    #         # 检查电梯可预约的时间段
                    #         elev_start_2, elev_end_2, elev_ready_2 = elev.check_reserve(current_time, before, between,
                    #                                                                     from_floor_2)
                    #
                    #         wait_time_2 = max(elev_ready_2 - current_time - before, 0)
                    #         actual_time = elev_end_2 + after - current_time
                    #
                    #         new_path_info = {
                    #             "path": res["path"],  # 总路径
                    #             "start_time": current_time,  # 任务起始时间
                    #             "actual_time": actual_time,  # 总耗时
                    #             "part_time_1": 0,
                    #             "part_time_2": actual_time,
                    #             "wait_time_1": 0,  # 分段等待时间
                    #             "wait_time_2": wait_time_2,
                    #             "before": before,  # 各段耗时
                    #             "between_1": 0,
                    #             "transfer": 0,
                    #             "between": between,
                    #             "after": after,
                    #             "start_e1": None,  # 第一部电梯起点和终点
                    #             "end_e1": None,
                    #             "start_e2": start_e,  # 第二部电梯起点和终点
                    #             "end_e2": end_e,
                    #             "eid_1": None,  # 第一部电梯
                    #             "eid_2": eid,  # 第二部电梯
                    #             "type": "elevator",
                    #             "from_floor_1": 0,  # 楼层信息
                    #             "end_floor_1": 0,
                    #             "from_floor_2": from_floor_2,
                    #             "end_floor_2": end_floor_2,
                    #             "elev_start_1": 0,  # 电梯调度起始时间
                    #             "elev_end_1": 0,  # 电梯调度结束时间
                    #             "elev_start_2": elev_start_2,  # 电梯调度起始时间
                    #             "elev_end_2": elev_end_2,  # 电梯调度结束时间
                    #             "status": status
                    #         }
                    #
                    #         print("添加 电梯-楼梯 混合路径！")
                    #         alternative_routes.append({
                    #             "type": "elevator_stair",
                    #             "elevator_id": eid,
                    #             "path_info": new_path_info,
                    #             "part": part,
                    #             "status": status,
                    #             "part_time_1": actual_time,
                    #             "part_time_2": actual_time,
                    #             "original_actual_time": original_actual_time,
                    #             "description": f"使用电梯 {eid}",
                    #             "from_cache": False
                    #         })

                    # print("添加 楼梯 路径！")
                    alternative_routes.append({
                        "type": "stair",
                        "path_info": stair_route,
                        "status": status,
                        "part_time_1": stair_route["part_time_2"],
                        "part_time_2": stair_route["part_time_2"],
                        "original_actual_time": original_actual_time,
                        "wait_time": 0.0,
                        "description": f"楼梯路径",
                        "from_cache": True  # 标记来自缓存
                    })

        # 2. 其他电梯路径（排除被禁用的电梯）
        for elevator_id, elevator_route in path_results.items():
            # 跳过楼梯路径
            if elevator_id == "stair_1" or elevator_id == "stair_2":
                # print("跳过楼梯路径！")
                continue

            # 判断是 取药品/送药品 过程冲突
            if elevator_route.get("status") != status:
                # print("跳过非冲突路径！")
                continue

            sections = elevator_id.split("|")  # 可能是 ["eid"] 或 ["eid", "eid_1"]
            len_sections = len(sections)
            eid_1 = assignment[path_info]["eid_1"]
            eid_2 = assignment[path_info]["eid_2"]
            # 特殊键单独处理
            if (len_sections == 1 and forbidden_elevator_id == sections and part == 2) or \
                    (len_sections == 2 and forbidden_elevator_id == sections[part - 1] and
                     eid_1 == sections[0] and eid_2 == sections[1]):
                # 3. 保持当前电梯路径，等待直至电梯空闲
                original_actual_time = assignment[path_info]["actual_time"]
                new_assignment = self.calculate_wait_option(
                    assignment, status, part, robot_id2, start2, forbidden_elevator_id)
                # path_info = path_info_map[status]
                if part == 1:
                    # print("添加 电梯 等待路径！")
                    alternative_routes.append({
                        "type": "elevator",
                        "elevator_id": elevator_id,
                        "path_info": new_assignment[path_info],
                        "part": part,
                        "status": status,
                        "part_time_1": new_assignment[path_info]["part_time_1"],
                        "part_time_2": new_assignment[path_info]["part_time_2"],
                        "original_actual_time": original_actual_time,
                        "description": f"使用电梯 {elevator_id}",
                        "from_cache": False
                    })
                else:
                    # print("添加 电梯 等待路径！")
                    alternative_routes.append({
                        "type": "elevator",
                        "elevator_id": elevator_id,
                        "path_info": new_assignment[path_info],
                        "part": part,
                        "status": status,
                        "part_time_2": new_assignment[path_info]["part_time_2"],
                        "description": f"使用电梯 {elevator_id}",
                        "from_cache": False
                    })
                continue

            # 这是一个有效的替代电梯路径
            if elevator_id in assignment["selected"]:
                continue
            else:
                if len_sections == 2:
                    if part == 1 and eid_2 == sections[1]:
                        original_actual_time = elevator_route["actual_time"]
                        # print("添加 电梯 路径！")
                        alternative_routes.append({
                            "type": "elevator",
                            "elevator_id": elevator_id,
                            "path_info": elevator_route,
                            "part": part,
                            "status": status,
                            "part_time_1": elevator_route["part_time_1"],
                            "part_time_2": elevator_route["part_time_2"],
                            "original_actual_time": original_actual_time,
                            "description": f"使用电梯 {elevator_id}",
                            "from_cache": True
                        })
                    elif part == 2 and eid_1 == sections[0]:
                        # print("添加 电梯 路径！")
                        alternative_routes.append({
                            "type": "elevator",
                            "elevator_id": elevator_id,
                            "path_info": elevator_route,
                            "part": part,
                            "status": status,
                            "part_time_2": elevator_route["part_time_2"],
                            "description": f"使用电梯 {elevator_id}",
                            "from_cache": True
                        })
                elif len_sections == 1:
                    if elevator_route["elevator_stair"]:
                        original_actual_time = assignment[path_info]["actual_time"]
                        # print("添加 电梯-楼梯 混合路径！")
                        alternative_routes.append({
                            "type": "elevator_stair",
                            "elevator_id": elevator_id,
                            "path_info": elevator_route,
                            "part": part,
                            "status": status,
                            "part_time_1": 0,
                            "part_time_2": elevator_route["part_time_2"],
                            "original_actual_time": original_actual_time,
                            "description": f"使用电梯 {elevator_id}",
                            "from_cache": False
                        })
                    else:
                        # print("添加 电梯 路径！")
                        alternative_routes.append({
                            "type": "elevator",
                            "elevator_id": elevator_id,
                            "path_info": elevator_route,
                            "part": part,
                            "status": status,
                            "part_time_2": elevator_route["part_time_2"],
                            "description": f"使用电梯 {elevator_id}",
                            "from_cache": True
                        })

        if not alternative_routes:
            print("没有找到可添加的替代路线信息！！！")
        # else:
        # print(alternative_routes)

        part_time_key = f"part_time_{part}"
        # 过滤掉 new_part_time_1/2 < current_cost 的路径
        filtered_routes = [x for x in alternative_routes if x[part_time_key] >= current_cost]

        # 按 part_time_1/2 排序
        filtered_routes.sort(key=lambda x: x[part_time_key])

        # 更新 alternative_routes（如果需要）
        alternative_routes = filtered_routes

        # alternative_routes.sort(key=lambda x: x["actual_time"])

        print(f"  为机器人{assignment['robot_id']}找到{len(alternative_routes)}条替代路线:")
        # for i, route in enumerate(alternative_routes[:3]):  # 只显示最快的3条
        #     print(f"    {i + 1}. {route['description']}: {route['actual_time']:.2f}s")

        return alternative_routes

    def calculate_wait_option(self, assignment, status, part, robot_id2, start2, elevator_id):
        """
        计算等待电梯的方案

        有两种等待策略：
        1. 简单等待：等待到电梯当前任务结束（乐观）
        2. 智能等待：考虑电梯后续任务（悲观）

        参数:
        assignment: 需要调整的任务分配
        status: 0=取药阶段, 1=送药阶段
        part: 1=第一部电梯, 2=第二部电梯
        start2: 冲突对方的电梯开始使用时间
        end2: 冲突对方的电梯结束使用时间
        elevator_id: 冲突的电梯ID
        current_assignments: 当前所有任务分配（可选）

        返回: 调整后的assignment
        """
        global simulated_elevator_reservations
        # robot = assignment["robot"]
        # task = assignment["task"]
        current_time = assignment["start_time"]

        new_assignment = copy.deepcopy(assignment)

        path_info_map = {
            0: "pick_path_info",
            1: "deliver_path_info"
        }

        # 从assignment的path_info中获取原计划的电梯使用时间
        path_info = new_assignment[path_info_map[status]]
        original_part_time_2 = path_info["part_time_2"]

        # 确定这个assignment使用的是哪一部电梯（eid_1还是eid_2）
        if part == 1:
            current_task_start = path_info["start_time"]
            target_before_time_key = "before"
            target_elevator_time_key = "between_1"
            target_elevator_start_key = "elev_start_1"
            target_elevator_end_key = "elev_end_1"
            target_elevator_from_key = "from_floor_1"
            # target_elevator_to_key = "to_floor_1"
            target_wait_time_key = "wait_time_1"
            target_part_time_key = "part_time_1"
        elif part == 2:
            current_task_start = path_info["elev_end_1"]
            target_before_time_key = "transfer"
            target_elevator_time_key = "between"
            target_elevator_start_key = "elev_start_2"
            target_elevator_end_key = "elev_end_2"
            target_elevator_from_key = "from_floor_2"
            # target_elevator_to_key = "to_floor_2"
            target_wait_time_key = "wait_time_2"
            target_part_time_key = "part_time_2"
        else:
            print(f"错误：无效的part参数 {part}")
            return new_assignment

        # 原计划的电梯使用时间
        original_elevator_from = path_info.get(target_elevator_from_key, 0)
        # original_elevator_to = path_info.get(target_elevator_to_key, 0)
        # original_elevator_start = path_info.get(target_elevator_start_key, 0)
        original_elevator_end = path_info.get(target_elevator_end_key, 0)
        original_before_time = path_info.get(target_before_time_key, 0)
        elevator_travel_time = path_info.get(target_elevator_time_key, 0)
        # original_after_time = path_info.get("after", 0)

        # 获取电梯对象
        elevator = self.elevators[elevator_id]
        if not elevator:
            print(f"错误：电梯 {elevator_id} 不存在")
            return new_assignment

        # print(f"\n计算机器人 {robot.id} 在电梯 {elevator_id} 的等待方案:")
        # print(f"  原计划使用时间: {original_elevator_start:.2f}s - {original_elevator_end:.2f}s")
        # print(f"  冲突对方时间: {start2:.2f}s - {end2:.2f}s")

        # 简单等待 + 智能等待
        # 查找当前所有使用这个电梯的assignments
        if elevator_id in simulated_elevator_reservations:
            for reservation in simulated_elevator_reservations[elevator_id]:
                print(reservation)
                if reservation["robot_id"] == robot_id2 and reservation["start_time"] == start2:
                    res_start = reservation['start_time']
                    res_end = reservation['end_time']
                    res_from = reservation['from_floor']
                    res_to = reservation['to_floor']
                    res_robot_id = reservation['robot_id']

                    simple_wait_until = res_end  # 默认等待到冲突发生时 另一个电梯调度的结束时间
                    before = max(original_before_time + current_task_start - simple_wait_until, 0)
                    elev_start, elev_end, elev_ready = elevator.check_reserve(
                        simple_wait_until, before, elevator_travel_time, original_elevator_from, res_to)
                    if part == 1:
                        # 若第一部电梯发生等待，则影响第二部电梯的调度
                        part_time = elev_end - current_time
                        eid_2 = path_info["eid_2"]
                        from_floor_2 = path_info["from_floor_2"]
                        # end_floor_2 = path_info["end_floor_2"]
                        transfer = path_info["transfer"]
                        between = path_info["between"]
                        after = path_info["after"]

                        elev_start_2, elev_end_2, elev_ready_2 = self.elevators[eid_2].check_reserve(
                            elev_end, transfer, between, from_floor_2)
                        wait_time_2 = max(elev_start_2 - elev_end - transfer, 0)
                        part_time_2 = elev_end_2 + after - elev_end

                        path_info["part_time_2"] = part_time_2
                        path_info["elev_start_2"] = elev_start_2
                        path_info["elev_end_2"] = elev_end_2
                        path_info["wait_time_2"] = wait_time_2
                    else:
                        part_time = original_part_time_2 + elev_end - original_elevator_end
                    wait_time = max(elev_ready - current_task_start - original_before_time, 0)

                    path_info[target_part_time_key] = part_time
                    path_info[target_elevator_start_key] = elev_start
                    path_info[target_elevator_end_key] = elev_end
                    path_info[target_wait_time_key] = wait_time
                    break

        new_assignment[path_info_map[status]] = path_info

        return new_assignment

    def _resolve_stair_conflicts(self, assignments: List[dict]):
        resolved_assignments = copy.copy(assignments)

        path_info_map = {
            0: "pick_path_info",
            1: "deliver_path_info"
        }

        for assignments in resolved_assignments:
            for status in [0, 1]:
                path_info = assignments[path_info_map[status]]
                if has_stairs(path_info):
                    stair_pairs = []
                    path = path_info["path"]
                    for i in range(len(path) - 1):
                        current_node = path[i]
                        next_node = path[i + 1]
                        if 'Stair' in current_node and 'Stair' in next_node:
                            stair_pairs.append((current_node, next_node))

        return resolved_assignments

    def calculate_tasks_total_time(self, assignments: List[dict]):
        tasks_total_time = 0.0
        for assignment in assignments:
            pick_path_info = assignment["pick_path_info"]
            deliver_path_info = assignment["deliver_path_info"]

            task_total_time = pick_path_info["actual_time"] + deliver_path_info["actual_time"]
            tasks_total_time += task_total_time

        return tasks_total_time

    def _execute_assignments(self, assignments: List[dict], current_time: float):
        """
        执行最终的任务分配
        """
        global simulated_elevator_reservations
        simulated_elevator_reservations = {}  # 重置为空字典

        path_info_map = {
            0: "pick_path_info",
            1: "deliver_path_info"
        }

        print(f"\n=== 执行任务分配 ===")

        for assignment in assignments:
            robot = self.find_robot(assignment["robot_id"])
            task = assignment["task"]
            for status in [0, 1]:
                path_info_str = path_info_map[status]
                path_info = assignment[path_info_str]

                # 为最佳路径预约电梯
                if path_info["type"] == "elevator":
                    eid_1 = path_info["eid_1"]
                    if eid_1:
                        elev_1 = self.elevators[eid_1]
                        from_floor_1 = int(path_info["start_e1"].split("_")[0])
                        to_floor_1 = int(path_info["end_e1"].split("_")[0])
                        reserve_start_abs = path_info["elev_start_1"]
                        reserve_end_abs = path_info["elev_end_1"]
                        elev_1.reserve(  # 为最佳路径预约电梯
                            start_time=reserve_start_abs,
                            end_time=reserve_end_abs,
                            from_floor=from_floor_1,
                            to_floor=to_floor_1,
                            robot_id=robot.id
                        )
                    eid_2 = path_info["eid_2"]
                    elev_2 = self.elevators[eid_2]
                    from_floor_2 = int(path_info["start_e2"].split("_")[0])
                    to_floor_2 = int(path_info["end_e2"].split("_")[0])
                    reserve_start_abs = path_info["elev_start_2"]
                    reserve_end_abs = path_info["elev_end_2"]
                    elev_2.reserve(  # 为最佳路径预约电梯
                        start_time=reserve_start_abs,
                        end_time=reserve_end_abs,
                        from_floor=from_floor_2,
                        to_floor=to_floor_2,
                        robot_id=robot.id
                    )

            # 更新机器人状态
            # robot.position = task.target

            pick_path_info = assignment[path_info_map[0]]
            deliver_path_info = assignment[path_info_map[1]]

            path1 = pick_path_info["path"]
            path2 = deliver_path_info["path"]
            path_start_time = assignment["start_time"]
            path_total_time = pick_path_info["actual_time"] + deliver_path_info["actual_time"]
            # robot.available_time = assignment["end_time"]
            wait_time_1 = pick_path_info.get("wait_time_1", 0.0)
            wait_time_2 = pick_path_info.get("wait_time_2", 0.0)
            wait_time_3 = deliver_path_info.get("wait_time_1", 0.0)
            wait_time_4 = deliver_path_info.get("wait_time_2", 0.0)
            robot.add_task(
                task_id=task.id,
                start_time=path_start_time,
                finish_time=path_start_time + path_total_time,
                path1=path1,
                path2=path2,
                actual_time1=pick_path_info["actual_time"],
                actual_time2=deliver_path_info["actual_time"],
                wait_time_1=wait_time_1,
                wait_time_2=wait_time_2,
                wait_time_3=wait_time_3,
                wait_time_4=wait_time_4
            )

            robot.get_sorted_tasks()
            # print(robot.task_list)
            # robot.current_position = get_coordinates_from_node(robot.position)

            print(
                f"机器人 {robot.id} 执行任务 {task.id}: {assignment['start_time']:.2f}s - {assignment['end_time']:.2f}s")


    def find_robot(self, robot_id):
        for robot in self.robots:
            if robot.id == robot_id:
                return robot

    def find_feasible_robots(self, task):
        """返回技能匹配且电量充足的机器人列表。"""
        if not hasattr(self, "robots") or not isinstance(self.robots, list):
            return []

        return [
            r for r in self.robots
            if r.skill == task.skill and isinstance(
                r.expected_charge, (int, float)) and r.expected_charge >= MIN_CHARGE_THRESHOLD and not r.is_charging
        ]


def get_robot_status_real_time(batch_scheduler, current_timestamp=None):
    """
    获取当前所有机器人状态，输出格式：
    posionX/Y/Z 是实时坐标
    增加 running_time 和 total_time
    """
    if current_timestamp is None:
        current_timestamp = time.time()

    # 使用相对时间 now，与终端 loop 一致
    now = current_timestamp - batch_scheduler.start_time

    data_list = []

    for r in batch_scheduler.robots:
        pos_x, pos_y, pos_z = r.current_position

        # 状态判断
        status_val = 0 if now >= r.available_time else 1
        robot_type_val = 1 if r.skill.lower() == "dog" else 2
        robot_name = "Dog" + str(r.id) if robot_type_val == 1 else "Human" + str(r.id)

        # print(r.task_list)

        data_list.append({
            "robotId": str(r.id),
            "robotName": robot_name,
            "robotType": robot_type_val,
            "status": status_val,
            "posionX": round(pos_x, 2),
            "posionY": round(pos_y, 2),
            "posionZ": round(pos_z, 2),
            "running_time": round(r.running_time, 2),  # 当前任务已运行时间
            "total_time": round(r.path_total_time, 2),  # 当前任务总运行时间
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
        Task(0, "dog", "9_2_B", "3_5_A", 3),
        Task(1, "human", "3_5_A", "9_2_B", 3),
        Task(2, "dog", "6_3_F", "6_3_C", 3),
        Task(3, "human", "3_7_D", "3_7_D", 3)
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
    print("输入任务格式：<skill> <priority> <start_position> <target_position>，例如：dog 4_3_A 6_3_G")
    print("输入 'batch <skill> <priority> <start> <target> <skill> <priority> <start> <target> ...' 一次性调度多个任务")
    print("输入 'exit' 退出系统")
    print("输入 'robot' 查看机器人状态")
    print("----------------------------------")

    while True:
        prompt = "调度系统 > "
        user_input = input(prompt).strip()

        # 退出系统
        if user_input.lower() == "exit":
            now = time.time() - batch_scheduler.start_time
            print(f"\n系统运行时间: {now:.2f}秒")
            print("退出调度系统")
            break

        # 查看机器人状态
        elif user_input.lower() == "robot":
            now = time.time() - batch_scheduler.start_time
            print(f"\n系统运行时间: {now:.2f}秒")
            print("\n--- 机器人状态 ---")
            status_data = get_robot_status_real_time(batch_scheduler)
            print(json.dumps(status_data, indent=4, ensure_ascii=False))
            print("------------------\n")
            continue

        # 批量调度命令
        elif user_input.startswith("batch "):
            # 解析批量任务
            parts = user_input.split()[1:]  # 跳过 ‘batch’
            if len(parts) % 4 != 0:
                print("格式错误，请确保每个任务都有对应的机器人类型、优先级和取送过程的目标位置")
                # print("示例：batch dog 6_3_G human 4_3_A")
                print("示例：batch dog 1 4_3_A 6_2_G human 2 4_3_A 6_2_G")
                continue

            batch_tasks = []
            for i in range(0, len(parts), 4):
                skill = parts[i]
                priority = int(parts[i + 1])
                start = parts[i + 2]
                target = parts[i + 3]
                batch_tasks.append(Task(task_counter, skill, start, target, priority))
                task_counter += 1

            # for task in batch_tasks:
            #     print(task.target, task.skill)

            # 确定最佳任务顺序
            # print("\n确定最佳任务顺序...")
            # best_order, _ = batch_scheduler.find_optimal_schedule(robots, batch_tasks)

            # 执行批量调度
            print("\n执行批量调度...")
            assignments = batch_scheduler.schedule_batch(batch_tasks)
            # assignments = batch_scheduler.schedule_batch(best_order)
            for assignment in assignments:
                print(f"任务 {assignment['task'].id} 分配给机器人 {assignment['robot_id']}")

            # print(f"self.robots 内存地址: {id(batch_scheduler.robots)}")
            # for robot in batch_scheduler.robots:
            #     print(f"Robot {robot.id} 内存地址: {id(robot)}, task_list: {robot.task_list}")
            #     print(robot.current_position)

            continue

        # # 单个任务处理
        # elif len(user_input.split()) == 3:
        #     now = time.time() - batch_scheduler.start_time
        #     skill, start, target = user_input.split()
        #     task = Task(task_counter, skill, start, target)
        #     result = batch_scheduler.assign_task(task, now)
        #
        #     if "error" in result:
        #         print(f"[!] {result['error']}")
        #     else:
        #         print(f"\n系统运行时间: {now:.2f}秒")
        #         print(f"[OK] 任务分配成功: Robot {result['robot_id']}")
        #         print(f"    预计开始时间: {result['start_time']:.2f}秒")
        #         print(f"    预计完成时间: {result['end_time']:.2f}秒")
        #         task_counter += 1
        else:
            print("格式错误，请输入：batch <skill> <priority> <start_position> <target_position>（例如：dog 1 4_2_A 6_3_G）")


if __name__ == "__main__":
    # scheduler, _ = batch_scheduling_demo()
    # start_time = int(time.time())
    # 后续调用显式传递 scheduler
    # status = get_robot_status_real_time(scheduler, start_time)
    start_interactive_scheduler()
    # condition = check_condition("8_3_A", "1_2_D")
    # print(condition)

    # node = create_elevator_access_node("5_2_A", "3_E2")
    # print(node)

    # # 示例用法
    # res_1 = {"path": ["A", "B", "C", "D"]}
    # res_2 = {"path": ["D", "E", "F", "G"]}
    #
    # merged_path = merge_paths(res_1["path"], res_2["path"])
    # print(merged_path)  # 输出: ['A', 'B', 'C', 'D', 'E', 'F', 'G']
