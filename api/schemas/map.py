"""
地图相关 Pydantic 模型

定义地图配置、初始化的数据结构。
"""

from pydantic import BaseModel, Field, validator
from typing import Dict, Any, List, Optional


class MapInitializeRequest(BaseModel):
    """
    地图初始化请求模型 - 简化友好格式

    用户只需提供校园名称和机器人配置列表，系统会自动处理 YAML 文件。

    Examples:
        {
            "campus_name": "test_campus",
            "robots": [
                {"rid": 0, "skill": "dog", "position": "4_1_p1", "campus_name": "test_campus"},
                {"rid": 1, "skill": "human", "position": "5_1_p1", "campus_name": "test_campus"}
            ]
        }
    """
    campus_name: str = Field(
        ...,
        description="校园/区域名称，用于标识和存储地图数据",
        examples=["test_campus", "sandun", "zheshang"]
    )
    robots: List[Dict[str, Any]] = Field(
        ...,
        description="机器人配置列表。每个机器人需包含：rid(ID), skill(类型), position(位置), campus_name(校园)",
        min_length=1,
        examples=[[{"rid": 0, "skill": "dog", "position": "4_1_p1", "campus_name": "test_campus"}]]
    )


class MapInitializeResponse(BaseModel):
    """
    地图初始化响应模型

    Examples:
        {
            "schemaVersion": "v1",
            "messageType": "map_initialize_response",
            "status": "success",
            "campusName": "test_campus",
            "elevatorCount": 8,
            "robotCount": 4,
            "nodeCount": 21,
            "mqttConnected": true
        }
    """
    schemaVersion: str = Field(default="v1", description="响应版本")
    messageType: str = Field(default="map_initialize_response", description="消息类型")
    status: str = Field(default="success", description="状态：success/error")
    message: str = Field(default="Map initialized successfully", description="响应消息")
    campusName: str = Field(..., description="校园名称")
    elevatorCount: int = Field(default=0, description="电梯数量")
    robotCount: int = Field(default=0, description="机器人数量")
    nodeCount: int = Field(default=0, description="节点数量")
    mqttConnected: bool = Field(default=False, description="MQTT是否已连接")


class NodeInfo(BaseModel):
    """
    节点信息模型

    Examples:
        {
            "nodeName": "4_1_p1",
            "floor": 4,
            "building": 1,
            "localId": "p1",
            "nodeType": "room",
            "coordinates": {"x": 0.85, "y": -0.92, "z": 0}
        }
    """
    nodeName: str = Field(..., description="节点名称，格式：楼层_楼号_位置")
    floor: float = Field(..., description="楼层（支持半层如4.5）")
    building: int = Field(..., description="楼号")
    localId: str = Field(..., description="局部标识")
    nodeType: str = Field(default="room", description="节点类型：room/elevator/stair")
    coordinates: Optional[Dict[str, float]] = Field(None, description="坐标")


class NodesListResponse(BaseModel):
    """
    节点列表响应

    Examples:
        {
            "schemaVersion": "v1",
            "messageType": "nodes_list_response",
            "campusName": "test_campus",
            "nodes": [...],
            "totalCount": 21
        }
    """
    schemaVersion: str = Field(default="v1", description="响应版本")
    messageType: str = Field(default="nodes_list_response", description="消息类型")
    campusName: str = Field(..., description="校园名称")
    nodes: List[NodeInfo] = Field(default_factory=list, description="节点列表")
    totalCount: int = Field(default=0, description="节点总数")


class ElevatorInfo(BaseModel):
    """
    电梯信息模型

    Examples:
        {
            "elevatorId": "1_E1",
            "building": 1,
            "localId": "E1",
            "currentFloor": 1,
            "scheduleCount": 2
        }
    """
    elevatorId: str = Field(..., description="电梯ID，格式：楼号_E编号")
    building: int = Field(..., description="所属楼号")
    localId: str = Field(..., description="电梯编号")
    currentFloor: int = Field(default=1, description="当前楼层")
    scheduleCount: int = Field(default=0, description="预约数量")


class ElevatorsListResponse(BaseModel):
    """
    电梯列表响应

    Examples:
        {
            "schemaVersion": "v1",
            "messageType": "elevators_list_response",
            "elevators": [...],
            "totalCount": 8
        }
    """
    schemaVersion: str = Field(default="v1", description="响应版本")
    messageType: str = Field(default="elevators_list_response", description="消息类型")
    elevators: List[ElevatorInfo] = Field(default_factory=list, description="电梯列表")
    totalCount: int = Field(default=0, description="电梯总数")


class MapConfig(BaseModel):
    """
    地图配置模型（向后兼容）
    """
    speed_land: float = Field(default=1.5, gt=0, description="平地移动速度 (m/s)")
    speed_stair: float = Field(default=0.5, gt=0, description="楼梯移动速度 (m/s)")
    elevator_configs: Dict[str, Any] = Field(default_factory=dict, description="电梯配置")
    robot_configs: List[Dict[str, Any]] = Field(default_factory=list, description="机器人配置")
    graph_files: Dict[str, str] = Field(default_factory=dict, description="图配置文件路径")