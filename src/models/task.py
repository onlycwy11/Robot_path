"""
任务数据模型

定义任务类及其属性。
"""

from dataclasses import dataclass, field
from typing import Optional


@dataclass
class Task:
    """
    任务类

    Attributes:
        id: 任务编号
        skill: 需要的机器人技能 ("dog" 或 "human")
        start: 取货点节点
        target: 送货点节点
        priority: 优先级（数值越小优先级越高）
        pickup_duration: 取货耗时（秒）
        deliver_duration: 送货耗时（秒）
    """
    id: int
    skill: str
    start: str              # 取货点（药品位置）
    target: str             # 送货点（目标位置）
    priority: int = 3
    pickup_duration: float = 0.0
    deliver_duration: float = 0.0

    def __repr__(self) -> str:
        return (
            f"Task(id={self.id}, skill={self.skill}, "
            f"start={self.start}, target={self.target}, "
            f"priority={self.priority})"
        )

    def __str__(self) -> str:
        return f"Task {self.id}: {self.start} → {self.target}"


def create_task(
    task_id: int,
    skill: str,
    start: str,
    target: str,
    priority: int = 3,
    pickup_duration: float = 0.0,
    deliver_duration: float = 0.0
) -> Task:
    """
    创建任务的工厂函数

    Args:
        task_id: 任务编号
        skill: 机器人技能
        start: 取货点
        target: 送货点
        priority: 优先级
        pickup_duration: 取货耗时
        deliver_duration: 送货耗时

    Returns:
        Task 对象
    """
    return Task(
        id=task_id,
        skill=skill,
        start=start,
        target=target,
        priority=priority,
        pickup_duration=pickup_duration,
        deliver_duration=deliver_duration
    )