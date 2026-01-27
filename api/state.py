"""
全局状态管理模块

提供全局调度器状态的管理和访问，集成 MQTT 实时数据。
"""

import time
from typing import Optional, Dict

from src.schedulers.batch_scheduler import BatchScheduler
from src.core.graph import Graph
from src.utils.mqtt_handler import MQTTStatusHandler, RobotStatusMessage
from src.utils.config import DEFAULT_CONFIG, SystemConfig
from src.utils.logger import mqtt_logger


class GlobalState:
    """
    全局状态管理类

    管理调度器、图数据、MQTT 连接等全局资源。
    机器人状态优先从 MQTT 获取，无数据时返回空。

    Attributes:
        batch_scheduler: 批量调度器实例
        stair_graph: 楼梯图
        elevator_graphs: 电梯增强图字典
        campus_name: 当前校园名称
        mqtt_handler: MQTT 状态处理器
    """

    def __init__(self):
        self._batch_scheduler: Optional[BatchScheduler] = None
        self._stair_graph: Optional[Graph] = None
        self._elevator_graphs: dict = {}
        self._campus_name: Optional[str] = None
        self._start_time: Optional[float] = None
        self._initialized: bool = False
        self._task_counter: int = 0

        # MQTT 相关
        self._mqtt_handler: Optional[MQTTStatusHandler] = None
        self._mqtt_cache: Dict[int, RobotStatusMessage] = {}
        self._mqtt_connected: bool = False

    @property
    def batch_scheduler(self) -> Optional[BatchScheduler]:
        return self._batch_scheduler

    @property
    def stair_graph(self) -> Optional[Graph]:
        return self._stair_graph

    @property
    def elevator_graphs(self) -> dict:
        return self._elevator_graphs

    @property
    def campus_name(self) -> Optional[str]:
        return self._campus_name

    @property
    def initialized(self) -> bool:
        return self._initialized

    @property
    def task_counter(self) -> int:
        return self._task_counter

    @property
    def mqtt_connected(self) -> bool:
        """MQTT 是否已连接"""
        if self._mqtt_handler:
            return self._mqtt_handler.is_connected()
        return False

    @property
    def mqtt_cache(self) -> Dict[int, RobotStatusMessage]:
        """获取 MQTT 缓存"""
        return self._mqtt_cache.copy()

    def increment_task_counter(self) -> int:
        """增加任务计数器"""
        self._task_counter += 1
        return self._task_counter

    def start_mqtt(self, config: SystemConfig = DEFAULT_CONFIG):
        """
        启动 MQTT 连接

        Args:
            config: 系统配置
        """
        self._mqtt_handler = MQTTStatusHandler(
            config=config.mqtt,
            status_callback=self._on_mqtt_status_update
        )
        self._mqtt_handler.start()
        mqtt_logger.info(f"Started MQTT Handler, connecting to {config.mqtt.host}:{config.mqtt.port}")

    def stop_mqtt(self):
        """停止 MQTT"""
        if self._mqtt_handler:
            self._mqtt_handler.stop()
            self._mqtt_handler = None
        self._mqtt_cache.clear()

    def _on_mqtt_status_update(self, msg: RobotStatusMessage):
        """
        MQTT 状态更新回调

        更新缓存并同步到 Robot 对象
        """
        self._mqtt_cache[msg.robot_id] = msg

        # 同步到 Robot 对象
        if self._batch_scheduler:
            robot = self._batch_scheduler._find_robot_by_id(msg.robot_id)
            if robot:
                robot.update_from_mqtt_status(msg)

        mqtt_logger.debug(f"MQTT status updated for robot {msg.robot_id}")

    def get_robot_mqtt_status(self, robot_id: int) -> Optional[RobotStatusMessage]:
        """
        获取单个机器人的 MQTT 状态

        Args:
            robot_id: 机器人ID

        Returns:
            MQTT 状态消息，无数据时返回 None
        """
        return self._mqtt_cache.get(robot_id)

    def get_all_mqtt_status(self) -> Dict[int, RobotStatusMessage]:
        """
        获取所有机器人的 MQTT 状态

        Returns:
            MQTT 状态字典
        """
        return self._mqtt_cache.copy()

    def set_scheduler(
        self,
        batch_scheduler: BatchScheduler,
        stair_graph: Graph,
        elevator_graphs: dict,
        campus_name: str
    ):
        """设置调度器和图数据"""
        self._batch_scheduler = batch_scheduler
        self._stair_graph = stair_graph
        self._elevator_graphs = elevator_graphs
        self._campus_name = campus_name
        self._start_time = time.time()
        self._initialized = True

    def reset(self):
        """重置全局状态"""
        self.stop_mqtt()
        self._batch_scheduler = None
        self._stair_graph = None
        self._elevator_graphs = {}
        self._campus_name = None
        self._start_time = None
        self._initialized = False
        self._task_counter = 0
        self._mqtt_cache.clear()
        self._mqtt_connected = False

    def get_uptime(self) -> float:
        """获取运行时间"""
        if self._start_time:
            return time.time() - self._start_time
        return 0.0

    def get_robot_count(self) -> int:
        """获取机器人数量"""
        if self._batch_scheduler:
            return len(self._batch_scheduler.robots)
        return 0

    def get_elevator_count(self) -> int:
        """获取电梯数量"""
        return len(self._elevator_graphs)


# 全局状态实例
global_state = GlobalState()