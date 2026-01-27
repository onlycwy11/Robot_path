"""
系统配置参数模块

统一管理所有系统参数，包括速度、电梯时间、机器人阈值等。
支持从 .env 文件读取 MQTT 认证配置。
"""

import os
from dataclasses import dataclass, field
from typing import List, Optional

# 尝试加载 .env 文件
try:
    from dotenv import load_dotenv
    load_dotenv()
except ImportError:
    # 如果没有安装 python-dotenv，则手动读取环境变量
    pass


@dataclass(frozen=True)
class MQTTConfig:
    """
    MQTT 配置参数

    Attributes:
        host: MQTT 服务器地址
        port: MQTT 服务器端口
        username: MQTT 用户名
        password: MQTT 密码
        client_id: 客户端 ID
        topic_prefix: Topic 前缀
        status_topic: 状态消息 Topic 模板
        enabled: 是否启用 MQTT
        message_timeout: 消息超时时间（秒）
    """
    host: str = "localhost"
    port: int = 1883
    username: Optional[str] = None
    password: Optional[str] = None
    client_id: str = "robot_scheduler_client"
    topic_prefix: str = "pro"
    status_topic: str = "pro/robot/+/status_agg"
    enabled: bool = True
    message_timeout: int = 30


@dataclass(frozen=True)
class SystemConfig:
    """
    系统配置参数（不可变）

    Attributes:
        speed_land: 平地移动速度 (m/s)
        speed_stair: 楼梯移动速度 (m/s)
        speed_elevator_cm: 电梯移动速度 (cm/s)

        elevator_door_open: 电梯开门时间 (秒)
        elevator_door_close: 电梯关门时间 (秒)
        elevator_per_floor: 电梯跨层运行时间 (秒/层)

        min_charge_threshold: 机器人最低电量阈值 (%)
        charging_position: 充电点位置节点

        max_conflict_iterations: 冲突解决最大迭代次数
        elevator_xy_tolerance: 电梯坐标分组容差 (m)

        mqtt: MQTT 配置对象
        status_update_interval: 状态更新间隔 (秒)
    """
    # 速度参数
    speed_land: float = 1.5        # m/s
    speed_stair: float = 0.5       # m/s
    speed_elevator_cm: float = 200.0  # cm/s

    # 电梯时间参数
    elevator_door_open: float = 1.5   # 秒
    elevator_door_close: float = 1.5  # 秒
    elevator_per_floor: float = 1.75  # 秒/层

    # 机器人参数
    min_charge_threshold: float = 40.0  # %
    charging_position: str = "1_1_Left_1"

    # 调度参数
    max_conflict_iterations: int = 10
    elevator_xy_tolerance: float = 0.25  # m

    # MQTT 配置
    mqtt: MQTTConfig = field(default_factory=MQTTConfig)

    # 状态更新间隔
    status_update_interval: float = 0.5  # 秒

    # 保留旧的 mqtt_host/mqtt_port 以兼容旧代码
    @property
    def mqtt_host(self) -> str:
        return self.mqtt.host

    @property
    def mqtt_port(self) -> int:
        return self.mqtt.port

    def validate(self) -> List[str]:
        """
        验证配置完整性

        Returns:
            错误消息列表，空列表表示配置有效
        """
        errors = []

        # 验证速度参数
        if self.speed_land <= 0:
            errors.append(f"speed_land 必须大于 0: {self.speed_land}")
        if self.speed_stair <= 0:
            errors.append(f"speed_stair 必须大于 0: {self.speed_stair}")
        if self.speed_elevator_cm <= 0:
            errors.append(f"speed_elevator_cm 必须大于 0: {self.speed_elevator_cm}")

        # 验证电梯时间参数
        if self.elevator_door_open < 0:
            errors.append(f"elevator_door_open 不能为负数: {self.elevator_door_open}")
        if self.elevator_door_close < 0:
            errors.append(f"elevator_door_close 不能为负数: {self.elevator_door_close}")
        if self.elevator_per_floor < 0:
            errors.append(f"elevator_per_floor 不能为负数: {self.elevator_per_floor}")

        # 验证电量阈值
        if self.min_charge_threshold < 0 or self.min_charge_threshold > 100:
            errors.append(f"min_charge_threshold 范围错误 (0-100): {self.min_charge_threshold}")

        # 验证充电位置
        if not self.charging_position:
            errors.append("charging_position 不能为空")

        # 验证调度参数
        if self.max_conflict_iterations < 1:
            errors.append(f"max_conflict_iterations 必须至少为 1: {self.max_conflict_iterations}")
        if self.elevator_xy_tolerance < 0:
            errors.append(f"elevator_xy_tolerance 不能为负数: {self.elevator_xy_tolerance}")

        # 验证状态更新间隔
        if self.status_update_interval <= 0:
            errors.append(f"status_update_interval 必须大于 0: {self.status_update_interval}")

        # 验证 MQTT 配置
        if self.mqtt.enabled:
            if not self.mqtt.host:
                errors.append("MQTT enabled but host is empty")
            if self.mqtt.port <= 0 or self.mqtt.port > 65535:
                errors.append(f"MQTT port 范围错误 (1-65535): {self.mqtt.port}")
            if self.mqtt.message_timeout <= 0:
                errors.append(f"MQTT message_timeout 必须大于 0: {self.mqtt.message_timeout}")

        return errors

    def is_valid(self) -> bool:
        """检查配置是否有效"""
        return len(self.validate()) == 0

    def elevator_time_for_floors(self, delta_floor: int) -> float:
        """
        计算电梯跨层耗时

        公式: 开门 + 关门 + 运行 + 开门
        = elevator_door_open + elevator_door_close + elevator_per_floor * n + elevator_door_open

        Args:
            delta_floor: 跨层数量

        Returns:
            电梯总耗时 (秒)
        """
        n = abs(int(delta_floor))
        if n <= 0:
            return 0.0
        return (
            self.elevator_door_open
            + self.elevator_door_close
            + self.elevator_per_floor * n
            + self.elevator_door_open
        )


def load_mqtt_config_from_env() -> MQTTConfig:
    """
    从环境变量加载 MQTT 配置

    Returns:
        MQTTConfig 配置对象
    """
    return MQTTConfig(
        host=os.getenv("MQTT_HOST", "localhost"),
        port=int(os.getenv("MQTT_PORT", "1883")),
        username=os.getenv("MQTT_USERNAME") or None,
        password=os.getenv("MQTT_PASSWORD") or None,
        client_id=os.getenv("MQTT_CLIENT_ID", "robot_scheduler_client"),
        topic_prefix=os.getenv("MQTT_TOPIC_PREFIX", "pro"),
        status_topic=os.getenv("MQTT_STATUS_TOPIC", "pro/robot/+/status_agg"),
        enabled=os.getenv("MQTT_ENABLED", "true").lower() == "true",
        message_timeout=int(os.getenv("MQTT_MESSAGE_TIMEOUT", "30"))
    )


def load_config_from_env() -> SystemConfig:
    """
    从环境变量加载完整系统配置

    Returns:
        SystemConfig 配置对象
    """
    mqtt_config = load_mqtt_config_from_env()

    return SystemConfig(
        speed_land=float(os.getenv("SPEED_LAND", "1.5")),
        speed_stair=float(os.getenv("SPEED_STAIR", "0.5")),
        mqtt=mqtt_config
    )


# 默认配置实例（从环境变量加载）
DEFAULT_CONFIG = load_config_from_env()


# 电梯时间预计算表（性能优化）
ELEVATOR_TIME_TABLE = {
    n: DEFAULT_CONFIG.elevator_time_for_floors(n) for n in range(1, 10)
}


def get_elevator_time_from_table(delta_floor: int) -> float:
    """从预计算表获取电梯时间，超出范围则动态计算"""
    n = abs(int(delta_floor))
    if n in ELEVATOR_TIME_TABLE:
        return ELEVATOR_TIME_TABLE[n]
    return DEFAULT_CONFIG.elevator_time_for_floors(n)


def calculate_elevator_time_need(delta_floor: int, config: SystemConfig = DEFAULT_CONFIG) -> float:
    """计算电梯跨层耗时（兼容旧函数接口）"""
    return config.elevator_time_for_floors(delta_floor)