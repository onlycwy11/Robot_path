"""
MQTT 状态对接测试脚本

测试 MQTT 状态消息与机器人状态的对接。
"""

import time
import json
import threading
from typing import Dict, List

# 导入模块
from src.utils.config import DEFAULT_CONFIG, load_config_from_env
from src.utils.mqtt_handler import (
    MQTTStatusHandler,
    MQTTStatusSimulator,
    RobotStatusMessage,
    create_mqtt_handler,
    create_mqtt_simulator
)
from src.models.robot import Robot


class RobotStatusManager:
    """
    机器人状态管理器

    统一管理 MQTT 状态消息和机器人状态更新。
    """

    def __init__(self, robots: List[Robot], config=None):
        self.robots = {r.id: r for r in robots}
        self.config = config or load_config_from_env()

        # MQTT 处理器
        self._mqtt_handler: MQTTStatusHandler = None
        self._mqtt_simulator: MQTTStatusSimulator = None

    def start_mqtt(self):
        """启动 MQTT 连接"""
        def on_status_update(status_msg: RobotStatusMessage):
            self._handle_status_update(status_msg)

        self._mqtt_handler = create_mqtt_handler(
            self.config,
            status_callback=on_status_update
        )
        self._mqtt_handler.start()

        print(f"[StatusManager] MQTT started, connected={self._mqtt_handler.is_connected()}")

    def start_simulator(self):
        """启动模拟器（无真实 MQTT 时使用）"""
        def on_simulated_update(status_msg: RobotStatusMessage):
            self._handle_status_update(status_msg)

        robot_ids = list(self.robots.keys())
        self._mqtt_simulator = create_mqtt_simulator(
            robot_ids,
            update_callback=on_simulated_update
        )
        self._mqtt_simulator.start(interval=1.0)

        print(f"[StatusManager] Simulator started for robots: {robot_ids}")

    def _handle_status_update(self, status_msg: RobotStatusMessage):
        """处理状态更新"""
        robot_id = status_msg.robot_id
        if robot_id in self.robots:
            robot = self.robots[robot_id]
            robot.update_from_mqtt_status(status_msg)

    def stop(self):
        """停止所有连接"""
        if self._mqtt_handler:
            self._mqtt_handler.stop()
        if self._mqtt_simulator:
            self._mqtt_simulator.stop()

    def get_all_robot_status(self) -> Dict[int, Dict]:
        """获取所有机器人状态"""
        status = {}
        for rid, robot in self.robots.items():
            mqtt_summary = robot.get_mqtt_status_summary()
            status[rid] = {
                "id": rid,
                "position": robot.current_position,
                "battery": robot.charge,
                "is_charging": robot.is_charging,
                "mqtt": mqtt_summary
            }
        return status


def test_mqtt_connection():
    """测试 MQTT 连接"""
    print("=== MQTT 连接测试 ===")

    config = load_config_from_env()
    print(f"MQTT 配置:")
    print(f"  Host: {config.mqtt.host}")
    print(f"  Port: {config.mqtt.port}")
    print(f"  Username: {config.mqtt.username}")
    print(f"  Topic: {config.mqtt.status_topic}")
    print(f"  Enabled: {config.mqtt.enabled}")

    # 创建测试机器人
    robots = [
        Robot(12, "dog", "1_1_A"),
        Robot(13, "dog", "1_1_A"),
    ]

    # 创建状态管理器
    manager = RobotStatusManager(robots, config)

    # 尝试连接真实 MQTT
    if config.mqtt.enabled and config.mqtt.username:
        print("\n尝试连接真实 MQTT...")
        manager.start_mqtt()
    else:
        print("\n使用模拟器...")
        manager.start_simulator()

    # 运行 10 秒
    print("\n运行 10 秒，监控状态更新...")
    for i in range(10):
        time.sleep(1)
        status = manager.get_all_robot_status()
        for rid, s in status.items():
            pos = s["position"]
            print(
                f"  [{i+1}s] Robot {rid}: "
                f"pos=({pos[0]:.1f}, {pos[1]:.1f}), "
                f"battery={s['battery']:.0f}%"
            )

    # 停止
    manager.stop()
    print("\n测试完成")


def test_simulated_message():
    """测试模拟消息解析"""
    print("\n=== 模拟消息解析测试 ===")

    # 模拟 JSON 消息
    test_json = '''
    {
      "schemaVersion": "v1",
      "messageType": "robot_status_agg",
      "publishedAt": "2026-04-22T14:30:01+08:00",
      "heartbeatSeq": 1024,
      "intervalSec": 1,
      "robot": {
        "id": 12,
        "sn": "YJ-001",
        "name": "元杰一号",
        "brand": "yj",
        "model": "go2"
      },
      "workspace": {
        "bound": true,
        "id": 3,
        "name": "住院部A区",
        "code": "WS202604221430ABCD"
      },
      "platform": {
        "type": "yj",
        "appId": 8,
        "appName": "元杰生产环境",
        "topicPrefix": "pro"
      },
      "status": {
        "onlineStatus": "online",
        "taskStatus": "busy",
        "robotState": "busy",
        "battery": 87,
        "isCharging": false,
        "isMoving": true,
        "temperature": 42.6
      },
      "location": {
        "mapName": "floor_3",
        "position": {
          "x": 1.2,
          "y": 3.4,
          "z": 0,
          "yaw": 90
        }
      },
      "heartbeat": {
        "sourceTime": "2026-04-22 14:30:00",
        "lastStateAt": "2026-04-22T14:30:00+08:00",
        "stateAgeMs": 850,
        "isStale": false
      }
    }
    '''

    # 解析
    from src.utils.mqtt_handler import MQTTStatusHandler
    handler = MQTTStatusHandler(DEFAULT_CONFIG.mqtt)
    data = json.loads(test_json)
    msg = handler._parse_status_message(data)

    print(f"解析结果:")
    print(f"  Robot ID: {msg.robot_id}")
    print(f"  Robot Name: {msg.robot_name}")
    print(f"  Battery: {msg.battery}%")
    print(f"  Position: ({msg.position_x}, {msg.position_y}, {msg.position_z})")
    print(f"  State: {msg.robot_state}")
    print(f"  Online: {msg.online_status}")

    # 对接到机器人
    robot = Robot(12, "dog", "1_1_A")
    msg.received_at = time.time()
    robot.update_from_mqtt_status(msg)

    print(f"\n机器人状态更新后:")
    print(f"  current_position: {robot.current_position}")
    print(f"  charge: {robot.charge}%")
    print(f"  is_charging: {robot.is_charging}")


if __name__ == "__main__":
    test_simulated_message()
    test_mqtt_connection()