"""
系统相关 Pydantic 模型

定义系统状态、健康检查、错误响应的数据结构。
"""

from pydantic import BaseModel, Field
from typing import Optional, Dict, Any


class HealthCheckResponse(BaseModel):
    """
    健康检查响应模型

    Examples:
        {"schemaVersion": "v1", "messageType": "health_check", "status": "healthy", "mqttConnected": true}
    """
    schemaVersion: str = Field(default="v1", description="响应版本")
    messageType: str = Field(default="health_check", description="消息类型")
    status: str = Field(default="healthy", description="系统状态：healthy/degraded/error")
    mqttConnected: bool = Field(default=False, description="MQTT是否已连接")
    schedulerInitialized: bool = Field(default=False, description="调度器是否已初始化")
    timestamp: float = Field(..., description="当前时间戳")


class SystemStatusResponse(BaseModel):
    """
    系统状态响应模型

    Examples:
        {
            "schemaVersion": "v1",
            "initialized": true,
            "campusName": "test_campus",
            "uptimeSeconds": 120.5,
            "robotCount": 4,
            "mqttConnected": true
        }
    """
    schemaVersion: str = Field(default="v1", description="响应版本")
    messageType: str = Field(default="system_status", description="消息类型")
    initialized: bool = Field(..., description="是否已初始化")
    campusName: Optional[str] = Field(None, description="校园名称")
    uptimeSeconds: float = Field(default=0.0, description="运行时间（秒）")
    uptimeHuman: str = Field(default="0.0s", description="运行时间（人类可读）")
    robotCount: int = Field(default=0, description="机器人数量")
    elevatorCount: int = Field(default=0, description="电梯数量")
    mqttConnected: bool = Field(default=False, description="MQTT是否已连接")
    mqttHost: Optional[str] = Field(None, description="MQTT服务器地址")
    mqttPort: Optional[int] = Field(None, description="MQTT端口")


class ResetResponse(BaseModel):
    """
    重置响应模型

    Examples:
        {"schemaVersion": "v1", "messageType": "reset_response", "status": "success", "message": "System reset"}
    """
    schemaVersion: str = Field(default="v1", description="响应版本")
    messageType: str = Field(default="reset_response", description="消息类型")
    status: str = Field(default="success", description="状态")
    message: str = Field(default="System reset successfully", description="响应消息")


class RootResponse(BaseModel):
    """
    根路径响应模型

    Examples:
        {"message": "Welcome to Robot Cluster Scheduler", "docs": "/docs", "version": "2.0.0"}
    """
    schemaVersion: str = Field(default="v1", description="响应版本")
    messageType: str = Field(default="root_response", description="消息类型")
    message: str = Field(default="Welcome to Robot Cluster Scheduler System", description="欢迎消息")
    docs: str = Field(default="/docs", description="API文档路径")
    redoc: str = Field(default="/redoc", description="ReDoc文档路径")
    version: str = Field(default="2.0.0", description="API版本")


class ErrorResponse(BaseModel):
    """
    错误响应模型

    Examples:
        {
            "schemaVersion": "v1",
            "messageType": "error_response",
            "errorCode": "INVALID_INPUT",
            "errorMessage": "Invalid robot configuration",
            "details": {"field": "skill", "reason": "must be dog or human"}
        }
    """
    schemaVersion: str = Field(default="v1", description="响应版本")
    messageType: str = Field(default="error_response", description="消息类型")
    errorCode: str = Field(..., description="错误码")
    errorMessage: str = Field(..., description="错误消息")
    details: Optional[Dict[str, Any]] = Field(None, description="错误详情")