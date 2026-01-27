"""
机器人相关 Pydantic 模型

定义机器人配置输入、状态输出的数据结构。
输入：简化友好格式
输出：MQTT robot_status_agg 标准格式
"""

from pydantic import BaseModel, Field, validator
from typing import List, Optional
from enum import Enum


# ==================== 输入模型（简化友好格式） ====================

class SkillEnum(str, Enum):
    """机器人技能类型"""
    dog = "dog"
    human = "human"


class RobotConfigInput(BaseModel):
    """
    机器人配置输入模型 - 简化友好格式

    用户只需提供基本信息，系统会自动处理。

    Examples:
        {"rid": 0, "skill": "dog", "position": "4_1_p1", "campus_name": "test_campus"}
        {"rid": 1, "skill": "human", "position": "5_1_p1", "campus_name": "sandun"}
    """
    rid: int = Field(
        ...,
        description="机器人ID，唯一标识，建议从0开始编号",
        examples=[0, 1, 2, 3]
    )
    skill: SkillEnum = Field(
        ...,
        description="机器人技能类型：dog（四足机器人）或 human（人形机器人）",
        examples=["dog", "human"]
    )
    position: str = Field(
        default="4_1_p1",
        description="初始位置节点，格式：楼层_楼号_位置。例如 4_1_p1 表示4楼1号楼p1位置",
        examples=["4_1_p1", "5_1_p1", "1_1_A", "4.5_1_p1"]
    )
    campus_name: str = Field(
        ...,
        description="所属校园/区域名称，必须与初始化时的 campus_name 一致",
        examples=["test_campus", "sandun", "zheshang"]
    )

    class Config:
        # 使用 rid 作为字段名，但也接受 id 别名
        populate_by_name = True


class RobotsConfigInput(BaseModel):
    """
    机器人配置列表

    Examples:
        {
            "robots": [
                {"rid": 0, "skill": "dog", "position": "4_1_p1", "campus_name": "test_campus"},
                {"rid": 1, "skill": "human", "position": "5_1_p1", "campus_name": "test_campus"}
            ]
        }
    """
    robots: List[RobotConfigInput] = Field(
        ...,
        description="机器人配置列表，至少需要1个机器人",
        min_length=1,
        examples=[[{"rid": 0, "skill": "dog", "position": "4_1_p1", "campus_name": "test_campus"}]]
    )


# ==================== 输出模型（MQTT 标准格式） ====================

class RobotInfo(BaseModel):
    """
    机器人基本信息

    对应 MQTT robot 字段
    """
    id: Optional[int] = Field(None, description="机器人ID")
    sn: Optional[str] = Field(None, description="机器人序列号")
    name: Optional[str] = Field(None, description="机器人名称")
    brand: Optional[str] = Field(None, description="机器人品牌")
    model: Optional[str] = Field(None, description="机器人型号")


class WorkspaceInfo(BaseModel):
    """
    工作空间信息

    对应 MQTT workspace 字段
    """
    id: Optional[int] = Field(None, description="工作空间ID")
    name: Optional[str] = Field(None, description="工作空间名称")
    code: Optional[str] = Field(None, description="工作空间编码")
    bound: Optional[bool] = Field(None, description="是否绑定工作空间")
    businessUUID: Optional[str] = Field(None, description="业务UUID")


class PlatformInfo(BaseModel):
    """
    平台信息

    对应 MQTT platform 字段
    """
    type: Optional[str] = Field(None, description="平台类型")
    appId: Optional[int] = Field(None, description="应用ID")
    appName: Optional[str] = Field(None, description="应用名称")
    topicPrefix: Optional[str] = Field(None, description="Topic前缀")


class StatusInfo(BaseModel):
    """
    状态信息

    对应 MQTT status 字段
    """
    onlineStatus: Optional[str] = Field(None, description="在线状态：online/offline")
    taskStatus: Optional[str] = Field(None, description="任务状态：idle/busy/error")
    robotState: Optional[str] = Field(None, description="机器人状态：idle/moving/error")
    battery: Optional[int] = Field(None, description="电量百分比 (0-100)", ge=0, le=100)
    isCharging: Optional[bool] = Field(None, description="是否正在充电")
    isMoving: Optional[bool] = Field(None, description="是否正在移动")
    temperature: Optional[float] = Field(None, description="温度（摄氏度）")


class Position3D(BaseModel):
    """
    三维位置坐标

    对应 MQTT location.position 字段
    """
    x: Optional[float] = Field(None, description="X坐标")
    y: Optional[float] = Field(None, description="Y坐标")
    z: Optional[float] = Field(None, description="Z坐标（高度）")
    yaw: Optional[float] = Field(None, description="偏航角（度）")


class LocationInfo(BaseModel):
    """
    位置定位信息

    对应 MQTT location 字段
    """
    mapName: Optional[str] = Field(None, description="地图名称")
    position: Optional[Position3D] = Field(None, description="三维坐标")


class HeartbeatInfo(BaseModel):
    """
    心跳信息

    对应 MQTT heartbeat 字段
    """
    sourceTime: Optional[str] = Field(None, description="源时间")
    lastStateAt: Optional[str] = Field(None, description="最后状态时间")
    stateAgeMs: Optional[int] = Field(None, description="状态年龄（毫秒）")
    isStale: Optional[bool] = Field(None, description="数据是否过期")


class RobotStatusAgg(BaseModel):
    """
    机器人状态聚合消息 - MQTT robot_status_agg 标准格式

    数据来源：优先从 MQTT 实时数据获取。
    若 MQTT 无数据，则字段值为空（null）。

    Examples:
        {
            "schemaVersion": "v1",
            "messageType": "robot_status_agg",
            "robot": {"id": 0, "name": "Dog0", "brand": "simulator"},
            "status": {"battery": 87, "isCharging": false, "isMoving": true},
            "location": {"mapName": "floor_4", "position": {"x": 1.2, "y": 3.4, "z": 0}}
        }
    """
    schemaVersion: str = Field(default="v1", description="消息版本")
    messageType: str = Field(default="robot_status_agg", description="消息类型")
    publishedAt: Optional[str] = Field(None, description="发布时间")
    heartbeatSeq: Optional[int] = Field(None, description="心跳序号")
    intervalSec: Optional[int] = Field(None, description="间隔秒数")

    robot: Optional[RobotInfo] = Field(None, description="机器人基本信息")
    workspace: Optional[WorkspaceInfo] = Field(None, description="工作空间信息")
    platform: Optional[PlatformInfo] = Field(None, description="平台信息")
    status: Optional[StatusInfo] = Field(None, description="状态信息")
    location: Optional[LocationInfo] = Field(None, description="位置信息")
    heartbeat: Optional[HeartbeatInfo] = Field(None, description="心跳信息")


class RobotsStatusResponse(BaseModel):
    """
    机器人状态响应

    返回所有或单个机器人的 MQTT 标准格式状态
    """
    schemaVersion: str = Field(default="v1", description="响应版本")
    messageType: str = Field(default="robots_status_agg", description="消息类型")
    publishedAt: Optional[str] = Field(None, description="响应时间")
    robots: List[RobotStatusAgg] = Field(
        default_factory=list,
        description="机器人状态列表（MQTT格式）"
    )
    totalCount: int = Field(default=0, description="机器人总数")
    mqttConnected: bool = Field(default=False, description="MQTT是否已连接")


# ==================== 旧版兼容（向后兼容） ====================

class RobotStatusOutput(BaseModel):
    """机器人状态输出模型（旧版兼容，逐步废弃）"""
    robot_id: str
    robot_name: str
    robot_type: int
    status: int
    position_x: float
    position_y: float
    position_z: float
    running_time: float = 0.0
    total_time: float = 0.0
    battery: float = 100.0
    is_charging: bool = False