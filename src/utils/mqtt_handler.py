"""
MQTT 消息处理器模块

解析机器人状态消息并更新机器人状态。
"""

import json
import time
import threading
import re
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Any, Callable

import paho.mqtt.client as mqtt

from src.utils.config import MQTTConfig, SystemConfig, DEFAULT_CONFIG
from src.utils.exceptions import SchedulerError
from src.utils.logger import mqtt_logger


@dataclass
class RobotStatusMessage:
    """
    机器人状态消息数据结构

    对应 MQTT JSON 消息格式
    """
    schema_version: str = "v1"
    message_type: str = "robot_status_agg"
    published_at: str = ""
    heartbeat_seq: int = 0
    interval_sec: int = 1

    # 机器人信息
    robot_id: int = 0
    robot_sn: str = ""
    robot_name: str = ""
    robot_brand: str = ""
    robot_model: str = ""

    # 工作空间信息
    workspace_id: int = 0
    workspace_name: str = ""
    workspace_code: str = ""
    workspace_bound: bool = False

    # 平台信息
    platform_type: str = ""
    app_id: int = 0
    app_name: str = ""
    topic_prefix: str = ""

    # 状态信息
    online_status: str = "offline"
    task_status: str = "idle"
    robot_state: str = "idle"
    battery: int = 0
    is_charging: bool = False
    is_moving: bool = False
    temperature: float = 0.0

    # 位置信息
    map_name: str = ""
    position_x: float = 0.0
    position_y: float = 0.0
    position_z: float = 0.0
    position_yaw: float = 0.0

    # 心跳信息
    source_time: str = ""
    last_state_at: str = ""
    state_age_ms: int = 0
    is_stale: bool = False

    # 接收时间
    received_at: float = 0.0


class MQTTStatusHandler:
    """
    MQTT 状态消息处理器

    订阅机器人状态消息，解析后更新机器人状态。
    """

    def __init__(
        self,
        config: MQTTConfig,
        status_callback: Optional[Callable[[RobotStatusMessage], None]] = None
    ):
        self.config = config
        self.status_callback = status_callback

        # MQTT 客户端
        self._client: Optional[mqtt.Client] = None
        self._running = False
        self._connected = False

        # 机器人状态缓存
        self._robot_status_cache: Dict[int, RobotStatusMessage] = {}

        # 连接锁
        self._lock = threading.Lock()

    def start(self):
        """启动 MQTT 连接"""
        if not self.config.enabled:
            mqtt_logger.info("MQTT disabled in config")
            return

        self._running = True
        self._connect()

    def stop(self):
        """停止 MQTT 连接"""
        self._running = False
        if self._client:
            self._client.loop_stop()
            self._client.disconnect()
            self._client = None

    def _connect(self):
        """连接 MQTT 服务器"""
        self._client = mqtt.Client(client_id=self.config.client_id)

        # 设置认证
        if self.config.username and self.config.password:
            self._client.username_pw_set(
                self.config.username,
                self.config.password
            )

        # 设置回调
        self._client.on_connect = self._on_connect
        self._client.on_disconnect = self._on_disconnect
        self._client.on_message = self._on_message

        try:
            self._client.connect(
                self.config.host,
                self.config.port,
                keepalive=60
            )
            self._client.loop_start()
            mqtt_logger.info(f"Connecting to {self.config.host}:{self.config.port}")
        except Exception as e:
            mqtt_logger.error(f"Connection failed: {e}")
            self._connected = False

    def _on_connect(self, client, userdata, flags, rc):
        """连接成功回调"""
        self._connected = True
        mqtt_logger.info(f"Connected with result code {rc}")

        # 订阅状态 Topic
        client.subscribe(self.config.status_topic)
        mqtt_logger.debug(f"Subscribed to {self.config.status_topic}")

    def _reconnect(self):
        """安全重连：先清理旧客户端再连接"""
        if self._client:
            try:
                self._client.loop_stop()
                self._client.disconnect()
            except Exception:
                pass  # 忽略清理错误
            self._client = None

        # 短暂延迟后重新连接
        time.sleep(2)
        self._connect()

    def _on_disconnect(self, client, userdata, rc):
        """断开连接回调"""
        self._connected = False
        mqtt_logger.warning(f"Disconnected with result code {rc}")

        # 自动重连（使用安全的重连方法）
        if self._running and rc != 0:
            mqtt_logger.info("Unexpected disconnect, attempting reconnection...")
            threading.Thread(target=self._reconnect, daemon=True).start()

    def _on_message(self, client, userdata, msg):
        """消息回调"""
        try:
            payload = msg.payload.decode('utf-8')
            data = json.loads(payload)

            # 解析消息
            status_msg = self._parse_status_message(data)
            status_msg.received_at = time.time()

            # 提取机器人 ID 从 Topic
            robot_id = self._extract_robot_id_from_topic(msg.topic)
            if robot_id:
                status_msg.robot_id = robot_id

            # 缓存状态
            self._robot_status_cache[status_msg.robot_id] = status_msg

            # 调用回调
            if self.status_callback:
                self.status_callback(status_msg)

            # 打印日志
            mqtt_logger.debug(
                f"Robot {status_msg.robot_id} ({status_msg.robot_name}): "
                f"battery={status_msg.battery}%, "
                f"pos=({status_msg.position_x:.2f}, {status_msg.position_y:.2f})"
            )

        except json.JSONDecodeError as e:
            mqtt_logger.error(f"JSON decode error: {e}")
        except Exception as e:
            mqtt_logger.error(f"Message handling error: {e}")

    def _parse_status_message(self, data: Dict[str, Any]) -> RobotStatusMessage:
        """解析状态消息"""
        msg = RobotStatusMessage()

        # 基本信息
        msg.schema_version = data.get("schemaVersion", "v1")
        msg.message_type = data.get("messageType", "")
        msg.published_at = data.get("publishedAt", "")
        msg.heartbeat_seq = data.get("heartbeatSeq", 0)
        msg.interval_sec = data.get("intervalSec", 1)

        # 机器人信息
        robot = data.get("robot", {})
        msg.robot_id = robot.get("id", 0)
        msg.robot_sn = robot.get("sn", "")
        msg.robot_name = robot.get("name", "")
        msg.robot_brand = robot.get("brand", "")
        msg.robot_model = robot.get("model", "")

        # 工作空间
        workspace = data.get("workspace", {})
        msg.workspace_id = workspace.get("id", 0)
        msg.workspace_name = workspace.get("name", "")
        msg.workspace_code = workspace.get("code", "")
        msg.workspace_bound = workspace.get("bound", False)

        # 平台
        platform = data.get("platform", {})
        msg.platform_type = platform.get("type", "")
        msg.app_id = platform.get("appId", 0)
        msg.app_name = platform.get("appName", "")
        msg.topic_prefix = platform.get("topicPrefix", "")

        # 状态
        status = data.get("status", {})
        msg.online_status = status.get("onlineStatus", "offline")
        msg.task_status = status.get("taskStatus", "idle")
        msg.robot_state = status.get("robotState", "idle")
        msg.battery = status.get("battery", 0)
        msg.is_charging = status.get("isCharging", False)
        msg.is_moving = status.get("isMoving", False)
        msg.temperature = status.get("temperature", 0.0)

        # 位置
        location = data.get("location", {})
        msg.map_name = location.get("mapName", "")
        position = location.get("position", {})
        msg.position_x = position.get("x", 0.0)
        msg.position_y = position.get("y", 0.0)
        msg.position_z = position.get("z", 0.0)
        msg.position_yaw = position.get("yaw", 0.0)

        # 心跳
        heartbeat = data.get("heartbeat", {})
        msg.source_time = heartbeat.get("sourceTime", "")
        msg.last_state_at = heartbeat.get("lastStateAt", "")
        msg.state_age_ms = heartbeat.get("stateAgeMs", 0)
        msg.is_stale = heartbeat.get("isStale", False)

        return msg

    def _extract_robot_id_from_topic(self, topic: str) -> Optional[int]:
        """
        从 Topic 提取机器人 ID

        Topic 格式: {prefix}/robot/{robotId}/status_agg
        例如: pro/robot/12/status_agg
        """
        match = re.search(r'/robot/(\d+)/', topic)
        if match:
            return int(match.group(1))
        return None

    def get_robot_status(self, robot_id: int) -> Optional[RobotStatusMessage]:
        """获取指定机器人的最新状态"""
        return self._robot_status_cache.get(robot_id)

    def get_all_robot_status(self) -> Dict[int, RobotStatusMessage]:
        """获取所有机器人的最新状态"""
        return self._robot_status_cache.copy()

    def is_connected(self) -> bool:
        """检查是否已连接"""
        return self._connected


def create_mqtt_handler(
    config: SystemConfig = DEFAULT_CONFIG,
    status_callback: Optional[Callable[[RobotStatusMessage], None]] = None
) -> MQTTStatusHandler:
    """
    创建 MQTT 状态处理器

    Args:
        config: 系统配置
        status_callback: 状态更新回调函数

    Returns:
        MQTTStatusHandler 实例
    """
    return MQTTStatusHandler(config.mqtt, status_callback)