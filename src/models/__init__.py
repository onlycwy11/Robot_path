# src/models/__init__.py
"""
核心数据模型模块

包含:
- Robot: 机器人状态和任务管理
- Elevator: 电梯调度管理
- Stair: 楼梯使用管理
- Task: 任务定义
- path_info: 路径信息类型定义
"""

from src.models.robot import Robot, RobotTaskInfo
from src.models.elevator import Elevator, Stair, ElevatorScheduleEntry, init_elevators
from src.models.task import Task, create_task
from src.models.path_info import (
    # 新版 dataclasses
    PathSegment,
    ElevatorSegment,
    FullPathInfo,
    ElevatorUsageRecord,
    ConflictInfo,
    TaskAssignment,
    #旧版 TypedDict (向后兼容)
    PathInfo,
    PathSegments,
    Assignment,
    Conflict,
    ElevatorUsage,
    AlternativeRoute,
    # 转换函数
    from_legacy_path_info,
)

__all__ = [
    # Robot
    "Robot",
    "RobotTaskInfo",
    # Elevator
    "Elevator",
    "Stair",
    "ElevatorScheduleEntry",
    "init_elevators",
    # Task
    "Task",
    "create_task",
    # PathInfo - 新版 dataclasses
    "PathSegment",
    "ElevatorSegment",
    "FullPathInfo",
    "ElevatorUsageRecord",
    "ConflictInfo",
    "TaskAssignment",
    # PathInfo -旧版 TypedDict
    "PathInfo",
    "PathSegments",
    "Assignment",
    "Conflict",
    "ElevatorUsage",
    "AlternativeRoute",
    # 转换函数
    "from_legacy_path_info",
]