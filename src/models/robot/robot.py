"""
机器人核心类 - 组件化架构

整合 TaskManager 和 PositionTracker 组件，
支持 MQTT 实时状态更新。
"""

from __future__ import annotations

import json
import math
import threading
import time
from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional, Any, TYPE_CHECKING

import paho.mqtt.client as mqtt

from src.utils.config import DEFAULT_CONFIG, SystemConfig
from src.utils.constants import RobotType, RobotStatus
from src.utils.logger import robot_logger

from src.models.robot.task_manager import RobotTaskManager, RobotTaskInfo
from src.models.robot.position_tracker import RobotPositionTracker

if TYPE_CHECKING:
    from src.utils.mqtt_handler import RobotStatusMessage


class Robot:
    """
    机器人类 - 组件化架构

    使用 TaskManager 管理任务，PositionTracker 管理位置。
    支持 MQTT 实时状态同步。

    Attributes:
        id: 机器人编号
        skill: 技能类型 ("dog" 或 "human")
        position: 当前位置节点
        charge: 电量百分比
        available_time: 空闲时间（秒）
        campus_name: 校园名称
        config: 系统配置
    """

    def __init__(
        self,
        rid: int,
        skill: str,
        position: str,
        campus_name: str = "zheshang",
        config: SystemConfig = DEFAULT_CONFIG,
        enable_mqtt_charge_updates: bool = False
    ):
        self.id = rid
        self.skill = skill
        self.position = position
        self.campus_name = campus_name
        self.config = config

        # 线程安全锁
        self._lock = threading.Lock()

        # 初始化组件
        self._task_manager = RobotTaskManager(rid)
        self._position_tracker = RobotPositionTracker(rid, campus_name, config)
        self._position_tracker.set_initial_position(position)

        # 状态属性
        self.initial_time = 0
        self.charge = 100.0
        self.is_charging = False
        self.running_time = 0.0  # 已运行时间
        self.path_total_time = 0.0  # 路径总时间

        # MQTT 实时状态属性
        self._mqtt_status_valid = False
        self._mqtt_last_update_time = 0.0
        self._mqtt_position_x = 0.0
        self._mqtt_position_y = 0.0
        self._mqtt_position_z = 0.0
        self._mqtt_position_yaw = 0.0
        self._mqtt_battery = 100
        self._mqtt_is_moving = False
        self._mqtt_online = True
        self._mqtt_map_name = ""

        # 预期状态（调度计算用）
        self.expected_position = position
        self.available_time = 0.0
        self.expected_available_time = 0.0

        # MQTT 电量更新
        self.enable_mqtt_charge_updates = enable_mqtt_charge_updates
        self.mqtt_client: Optional[mqtt.Client] = None

        if self.enable_mqtt_charge_updates:
            self._start_mqtt_listener()

        # 后台状态更新线程
        self._running = True
        self._status_update_thread = threading.Thread(target=self._auto_update_status)
        self._status_update_thread.daemon = True
        self._status_update_thread.start()

    # ==================== 属性代理 ====================

    @property
    def task_list(self) -> Dict[int, RobotTaskInfo]:
        """代理到 TaskManager"""
        return self._task_manager.get_all_tasks()

    @property
    def current_position(self) -> Tuple[float, float, float]:
        """代理到 PositionTracker"""
        return self._position_tracker.current_position

    @current_position.setter
    def current_position(self, value: Tuple[float, float, float]):
        self._position_tracker.current_position = value

    # ==================== 时间设置 ====================

    def set_initial_time(self, start_time: float):
        """设置初始时间"""
        self.initial_time = start_time

    # ==================== MQTT 功能 ====================

    def _start_mqtt_listener(self):
        """启动 MQTT 电量监听"""
        def on_connect(client, userdata, flags, rc):
            client.subscribe(f"robot/{self.id}/charge")

        def on_message(client, userdata, msg):
            try:
                payload = json.loads(msg.payload.decode())
                new_charge = float(payload.get("charge_level", 100))

                if new_charge < 0:
                    new_charge = 0.0
                elif new_charge > 100.0:
                    new_charge = 100.0

                if new_charge < self.config.min_charge_threshold and not self.is_charging:
                    self.charge = 0.0
                    self.is_charging = True
                    self.position = self.config.charging_position
                    self._position_tracker.set_initial_position(self.config.charging_position)
                    self._task_manager.clear_tasks()

                elif self.is_charging:
                    if new_charge > self.charge:
                        self.charge = new_charge
                    if self.charge >= 100.0:
                        self.is_charging = False
                        self.charge = 100.0
                else:
                    self.charge = new_charge

            except Exception as e:
                robot_logger.warning(f"Error updating charge: {e}")

        try:
            self.mqtt_client = mqtt.Client()
            self.mqtt_client.on_connect = on_connect
            self.mqtt_client.on_message = on_message
            self.mqtt_client.connect(
                self.config.mqtt_host,
                self.config.mqtt_port
            )
            self.mqtt_client.loop_start()
        except Exception as e:
            robot_logger.warning(f"MQTT init failed: {e}")
            self.enable_mqtt_charge_updates = False

    def set_mqtt_charge_updates(self, enable: bool):
        """动态设置 MQTT 电量更新"""
        if enable == self.enable_mqtt_charge_updates:
            return

        self.enable_mqtt_charge_updates = enable
        if enable:
            try:
                if not self.mqtt_client:
                    self._start_mqtt_listener()
                else:
                    self.mqtt_client.subscribe(f"robot/{self.id}/charge")
            except Exception as e:
                robot_logger.error(f"Enable MQTT failed: {e}")
                self.enable_mqtt_charge_updates = False
        else:
            if self.mqtt_client:
                self.mqtt_client.unsubscribe(f"robot/{self.id}/charge")

    def _use_mqtt_position(self) -> bool:
        """判断是否使用 MQTT 位置数据"""
        timeout = self.config.mqtt.message_timeout
        now = time.time()
        return (now - self._mqtt_last_update_time) < timeout

    def update_from_mqtt_status(self, status_msg: RobotStatusMessage):
        """从 MQTT 状态消息更新机器人状态（线程安全）"""
        if status_msg.robot_id != self.id:
            return

        with self._lock:
            self._mqtt_status_valid = True
            self._mqtt_last_update_time = status_msg.received_at

            # 更新位置
            self._mqtt_position_x = status_msg.position_x * 100
            self._mqtt_position_y = status_msg.position_y * 100
            self._mqtt_position_z = status_msg.position_z * 100
            self._mqtt_position_yaw = status_msg.position_yaw
            self._mqtt_map_name = status_msg.map_name

            # 更新电量
            self._mqtt_battery = status_msg.battery
            self.charge = float(status_msg.battery)

            # 更新状态
            self._mqtt_is_moving = status_msg.is_moving
            self._mqtt_online = status_msg.online_status == "online"
            self.is_charging = status_msg.is_charging

            # 更新实时位置
            self._position_tracker.current_position = (
                self._mqtt_position_x,
                self._mqtt_position_y,
                self._mqtt_position_z
            )

            if self.charge < self.config.min_charge_threshold:
                self.is_charging = True

        robot_logger.debug(
            f"MQTT status updated: battery={self.charge:.0f}%, "
            f"position=({self._mqtt_position_x:.1f}, {self._mqtt_position_y:.1f})"
        )

    def get_mqtt_status_summary(self) -> Dict[str, Any]:
        """获取 MQTT 状态摘要"""
        return {
            "robot_id": self.id,
            "valid": self._mqtt_status_valid,
            "last_update": self._mqtt_last_update_time,
            "position": {
                "x": self._mqtt_position_x,
                "y": self._mqtt_position_y,
                "z": self._mqtt_position_z,
                "yaw": self._mqtt_position_yaw
            },
            "battery": self._mqtt_battery,
            "is_moving": self._mqtt_is_moving,
            "online": self._mqtt_online,
            "map_name": self._mqtt_map_name
        }

    # ==================== 状态更新 ====================

    def _auto_update_status(self):
        """后台线程，定时更新机器人状态"""
        while self._running:
            current_time = time.time() - self.initial_time
            with self._lock:
                self._update_running_status(current_time)
            time.sleep(self.config.status_update_interval)

    def _update_running_status(self, current_time: float):
        """实时更新运行时间和当前位置（线程安全，需在锁内调用）"""
        # 优先使用 MQTT 实时状态
        if self._mqtt_status_valid and self._use_mqtt_position():
            self._position_tracker.current_position = (
                self._mqtt_position_x,
                self._mqtt_position_y,
                self._mqtt_position_z
            )
            self.charge = float(self._mqtt_battery)
            self.is_charging = self.charge < self.config.min_charge_threshold
            return

        if not self._task_manager.get_task_count() or self.is_charging:
            return

        # 更新为最后一个任务的状态
        sorted_tasks = self._task_manager.get_sorted_tasks()
        last_task_id = max(sorted_tasks.keys())
        last_task_info = sorted_tasks[last_task_id]
        self.position = last_task_info.path[-1]
        self.available_time = last_task_info.finish_time

        # 获取最早任务
        task_id = min(sorted_tasks.keys())
        task_info = sorted_tasks[task_id]

        if current_time >= task_info.finish_time:
            self._task_manager.remove_task(task_id)
            self._position_tracker.clear_path()
            return

        # 设置路径信息
        self._position_tracker.set_path(
            path=task_info.path,
            path1=task_info.path1,
            path2=task_info.path2,
            start_time=task_info.start_time,
            total_time=task_info.finish_time - task_info.start_time,
            pick_time=task_info.pick_time,
            deliver_time=task_info.deliver_time,
            wait_pair_1=task_info.wait_pair_1,
            wait_pair_2=task_info.wait_pair_2
        )

        # 更新位置
        running_time = current_time - task_info.start_time
        running_time = max(0.0, min(running_time, task_info.finish_time - task_info.start_time))
        self._position_tracker.update_position(running_time)

    # ==================== 任务管理（代理） ====================

    def add_task(
        self,
        task_id: int,
        start_time: float,
        finish_time: float,
        path1: List[str],
        path2: List[str],
        actual_time1: float,
        actual_time2: float,
        wait_time_1: float,
        wait_time_2: float,
        wait_time_3: float,
        wait_time_4: float,
        **kwargs
    ):
        """添加任务"""
        full_path = path1[:-1] + path2 if path1 and path2 and path1[-1] == path2[0] else path1 + path2

        self._task_manager.add_task(
            task_id=task_id,
            start_time=start_time,
            finish_time=finish_time,
            path=full_path,
            path1=path1,
            path2=path2,
            pick_time=actual_time1,
            deliver_time=actual_time2,
            wait_pair_1=(wait_time_1, wait_time_2),
            wait_pair_2=(wait_time_3, wait_time_4)
        )

    def remove_task(self, task_id: int) -> Optional[RobotTaskInfo]:
        """删除任务"""
        return self._task_manager.remove_task(task_id)

    def get_task(self, task_id: int) -> Optional[RobotTaskInfo]:
        """按任务 ID 查找"""
        return self._task_manager.get_task(task_id)

    def get_sorted_tasks(self) -> Dict[int, RobotTaskInfo]:
        """获取按开始时间排序的任务列表"""
        return self._task_manager.get_sorted_tasks()

    # ==================== 状态查询 ====================

    def is_available(self) -> bool:
        """检查是否可用（电量充足且不在充电）- 线程安全"""
        with self._lock:
            return (
                self.charge >= self.config.min_charge_threshold
                and not self.is_charging
            )

    def get_status(self) -> RobotStatus:
        """获取当前状态 - 线程安全"""
        with self._lock:
            current_time = time.time() - self.initial_time
            if current_time >= self.available_time:
                return RobotStatus.IDLE
            return RobotStatus.WORKING

    def get_type(self) -> RobotType:
        """获取机器人类型"""
        if self.skill.lower() == "dog":
            return RobotType.DOG
        return RobotType.HUMAN

    def calculate_electricity_consumption(self, full_path: List[str]) -> float:
        """
        计算预计电量消耗

        基于路径长度和类型计算消耗：
        - 电梯段：低消耗（距离 * 0.001）
        - 楼梯段：高消耗（距离 * 0.05）
        - 平地段：中等消耗（距离 * 0.02）

        Args:
            full_path: 完整节点路径列表

        Returns:
            预计电量消耗（百分比），最大不超过 100.0
        """
        if not full_path or len(full_path) < 2:
            return 0.0

        consumption = 0.0

        for i in range(len(full_path) - 1):
            node1 = full_path[i]
            node2 = full_path[i + 1]

            # 获取节点坐标
            from src.core.node import get_coordinates_from_node
            coord1 = get_coordinates_from_node(node1, self.campus_name)
            coord2 = get_coordinates_from_node(node2, self.campus_name)

            if not coord1 or not coord2:
                continue

            # 计算欧几里得距离（cm）
            distance = math.sqrt(
                (coord2[0] - coord1[0]) ** 2 +
                (coord2[1] - coord1[1]) ** 2 +
                (coord2[2] - coord1[2]) ** 2
            )

            # 转换为米
            distance_m = distance / 100.0

            # 根据路径类型计算消耗
            if self._is_elevator_segment(node1, node2):
                # 电梯段：低消耗（几乎不消耗电量）
                consumption += distance_m * 0.001
            elif self._is_stair_segment(node1, node2):
                # 楼梯段：高消耗（爬楼需要更多能量）
                consumption += distance_m * 0.05
            else:
                # 平地段：中等消耗
                consumption += distance_m * 0.02

        return min(consumption, 100.0)

    def _is_elevator_segment(self, node1: str, node2: str) -> bool:
        """判断是否为电梯段"""
        # 电梯节点通常包含 "E" 标识
        elevator_markers = ["E1", "E2", "E3", "E4", "E5", "E6"]
        for marker in elevator_markers:
            if marker in node1 and marker in node2:
                return True
        return False

    def _is_stair_segment(self, node1: str, node2: str) -> bool:
        """判断是否为楼梯段"""
        # 楼梯节点通常包含 "Stair" 标识
        stair_markers = ["Stair1", "Stair2", "Stair3", "Stair4", "Stair"]
        node1_lower = node1.lower()
        node2_lower = node2.lower()

        if "stair" in node1_lower and "stair" in node2_lower:
            return True
        return False

    # ==================== 生命周期 ====================

    def stop(self):
        """停止后台线程和 MQTT"""
        self._running = False
        if self.mqtt_client:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()

    def __del__(self):
        """清理资源"""
        self.stop()

    def __repr__(self) -> str:
        return f"Robot(id={self.id}, skill={self.skill}, position={self.position})"

    def __str__(self) -> str:
        status = self.get_status()
        return f"Robot {self.id} ({self.skill}): {self.position} [{status.name}]"