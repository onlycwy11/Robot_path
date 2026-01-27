"""
机器人任务管理器

管理机器人的任务列表、添加和移除任务。
"""

from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional
import time

from src.utils.logger import robot_logger


@dataclass
class RobotTaskInfo:
    """
    机器人任务信息

    Attributes:
        task_id: 任务编号
        start_time: 任务开始时间
        finish_time: 任务结束时间
        path: 完整路径
        path1: 取货路径
        path2: 送货路径
        pick_time: 取货耗时
        deliver_time: 送货耗时
        wait_pair_1: 第一段等待时间对
        wait_pair_2: 第二段等待时间对
    """
    task_id: int
    start_time: float
    finish_time: float
    path: List[str]
    path1: List[str]
    path2: List[str]
    pick_time: float
    deliver_time: float
    wait_pair_1: Tuple[float, float]
    wait_pair_2: Tuple[float, float]


class RobotTaskManager:
    """
    机器人任务管理器

    管理机器人的任务列表，提供添加、移除、查询任务的功能。
    """

    def __init__(self, robot_id: int):
        self.robot_id = robot_id
        self._tasks: Dict[int, RobotTaskInfo] = {}

    def add_task(
        self,
        task_id: int,
        start_time: float,
        finish_time: float,
        path: List[str],
        path1: List[str],
        path2: List[str],
        pick_time: float,
        deliver_time: float,
        wait_pair_1: Tuple[float, float],
        wait_pair_2: Tuple[float, float]
    ) -> RobotTaskInfo:
        """
        添加任务

        Args:
            task_id: 任务ID
            start_time: 开始时间
            finish_time: 结束时间
            path: 完整路径
            path1: 取货路径
            path2: 送货路径
            pick_time: 取货耗时
            deliver_time: 送货耗时
            wait_pair_1: 第一段等待时间
            wait_pair_2: 第二段等待时间

        Returns:
            RobotTaskInfo 任务信息对象
        """
        task_info = RobotTaskInfo(
            task_id=task_id,
            start_time=start_time,
            finish_time=finish_time,
            path=path,
            path1=path1,
            path2=path2,
            pick_time=pick_time,
            deliver_time=deliver_time,
            wait_pair_1=wait_pair_1,
            wait_pair_2=wait_pair_2
        )

        self._tasks[task_id] = task_info

        robot_logger.info(
            f"Robot {self.robot_id} added task {task_id}: "
            f"{start_time:.2f}s - {finish_time:.2f}s"
        )

        return task_info

    def remove_task(self, task_id: int) -> Optional[RobotTaskInfo]:
        """
        移除任务

        Args:
            task_id: 任务ID

        Returns:
            移除的任务信息，如果不存在则返回 None
        """
        if task_id in self._tasks:
            task_info = self._tasks.pop(task_id)
            robot_logger.info(f"Robot {self.robot_id} removed task {task_id}")
            return task_info
        return None

    def get_task(self, task_id: int) -> Optional[RobotTaskInfo]:
        """
        获取指定任务

        Args:
            task_id: 任务ID

        Returns:
            任务信息，如果不存在则返回 None
        """
        return self._tasks.get(task_id)

    def get_all_tasks(self) -> Dict[int, RobotTaskInfo]:
        """获取所有任务"""
        return self._tasks.copy()

    def get_sorted_tasks(self) -> Dict[int, RobotTaskInfo]:
        """获取按开始时间排序的任务列表"""
        sorted_items = sorted(
            self._tasks.items(),
            key=lambda x: x[1].start_time
        )
        self._tasks = dict(sorted_items)
        return self._tasks

    def get_active_task(self, current_time: float) -> Optional[RobotTaskInfo]:
        """
        获取当前正在执行的任务

        Args:
            current_time: 当前时间

        Returns:
            正在执行的任务，如果没有则返回 None
        """
        for task_info in self._tasks.values():
            if task_info.start_time <= current_time <= task_info.finish_time:
                return task_info
        return None

    def get_task_count(self) -> int:
        """获取任务数量"""
        return len(self._tasks)

    def clear_tasks(self) -> None:
        """清空所有任务"""
        self._tasks.clear()
        robot_logger.info(f"Robot {self.robot_id} cleared all tasks")

    def has_task(self, task_id: int) -> bool:
        """检查任务是否存在"""
        return task_id in self._tasks

    def __len__(self) -> int:
        return len(self._tasks)

    def __iter__(self):
        return iter(self._tasks.values())