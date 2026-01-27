"""
任务相关 Pydantic 模型

定义任务输入、输出的数据结构。
"""

from pydantic import BaseModel, Field
from typing import List


class SingleTaskInput(BaseModel):
    """
    单个任务输入模型

    Attributes:
        task_id: 任务ID
        skill: 任务类型（"dog" 或 "human"）
        priority: 任务优先级（数值越小优先级越高）
        start_position: 起点（格式：楼层_楼号_位置）
        target_position: 终点（格式：楼层_楼号_位置）
    """
    task_id: int = Field(..., description="任务ID")
    skill: str = Field(..., description="任务类型（dog/human）")
    priority: int = Field(..., ge=1, le=5, description="任务优先级（1-5）")
    start_position: str = Field(..., description="起点位置节点")
    target_position: str = Field(..., description="终点位置节点")


class BatchTaskInput(BaseModel):
    """
    批量任务输入模型

    Attributes:
        tasks: 任务列表
    """
    tasks: List[SingleTaskInput] = Field(..., description="任务列表")


class AssignmentOutput(BaseModel):
    """
    任务分配输出模型

    Attributes:
        task_id: 任务ID
        robot_id: 分配的机器人ID
        path: 预计路径（节点列表）
        estimated_time: 预估完成时间（秒）
    """
    task_id: int = Field(..., description="任务ID")
    robot_id: int = Field(..., description="分配的机器人ID")
    path: List[str] = Field(default_factory=list, description="完整路径节点列表")
    estimated_time: float = Field(..., ge=0, description="预估完成时间（秒）")


class TaskCancelResponse(BaseModel):
    """
    任务取消响应模型

    Attributes:
        task_id: 任务ID
        cancelled: 是否成功取消
        message: 响应消息
    """
    task_id: int
    cancelled: bool
    message: str