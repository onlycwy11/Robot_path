"""
协议接口定义模块

定义核心组件的抽象接口，用于类型检查和依赖注入。
"""

from __future__ import annotations

from typing import Protocol, Dict, List, Optional, Tuple, Any, TYPE_CHECKING

if TYPE_CHECKING:
    from src.models.task import Task
    from src.models.robot import Robot
    from src.models.elevator import Elevator
    from src.models.path_info import PathInfo, Assignment, FullPathInfo


class PathFinder(Protocol):
    """
    路径查找器接口

    定义路径计算的标准接口。
    """

    def find_shortest_path(
        self,
        start: str,
        end: str,
        **kwargs
    ) -> Optional[Dict[str, Any]]:
        """
        查找最短路径

        Args:
            start: 起点节点
            end: 终点节点

        Returns:
            路径信息字典，包含 path, time, distance 等
        """
        ...

    def find_path_with_elevator(
        self,
        start: str,
        end: str,
        start_floor: int,
        end_floor: int,
        **kwargs
    ) -> Optional[Dict[str, Any]]:
        """
        查找跨楼层路径（使用电梯）

        Args:
            start: 起点节点
            end: 终点节点
            start_floor: 起始楼层
            end_floor: 目标楼层

        Returns:
            路径信息字典
        """
        ...

    def get_path_time(
        self,
        path: List[str],
        **kwargs
    ) -> float:
        """
        计算路径耗时

        Args:
            path: 节点路径列表

        Returns:
            预计耗时（秒）
        """
        ...


class Scheduler(Protocol):
    """
    调度器接口

    定义任务调度和冲突解决的标准接口。
    """

    def schedule_task(
        self,
        task: Task,
        **kwargs
    ) -> Optional[Assignment]:
        """
        调度单个任务

        Args:
            task: 待调度任务

        Returns:
            任务分配信息，如果无法调度则返回 None
        """
        ...

    def schedule_batch(
        self,
        tasks: List[Task],
        **kwargs
    ) -> List[Assignment]:
        """
        批量调度任务

        Args:
            tasks: 待调度任务列表

        Returns:
            成功分配的任务列表
        """
        ...

    def find_available_robot(
        self,
        task: Task,
        current_time: float
    ) -> Optional[Robot]:
        """
        查找可用机器人

        Args:
            task: 任务对象
            current_time: 当前时间

        Returns:
            可用机器人，如果没有则返回 None
        """
        ...

    def resolve_conflicts(
        self,
        assignments: List[Assignment],
        **kwargs
    ) -> List[Assignment]:
        """
        解决电梯冲突

        Args:
            assignments: 任务分配列表

        Returns:
            解决冲突后的分配列表
        """
        ...


class ElevatorManager(Protocol):
    """
    电梯管理器接口

    定义电梯预约和状态管理的标准接口。
    """

    def reserve_elevator(
        self,
        elevator_id: str,
        robot_id: int,
        time_window: Tuple[float, float],
        **kwargs
    ) -> bool:
        """
        预约电梯

        Args:
            elevator_id: 电梯ID
            robot_id: 机器人ID
            time_window: 使用时间窗口 (start, end)

        Returns:
            预约是否成功
        """
        ...

    def check_availability(
        self,
        elevator_id: str,
        time_window: Tuple[float, float],
        **kwargs
    ) -> bool:
        """
        检查电梯可用性

        Args:
            elevator_id: 电梯ID
            time_window: 时间窗口

        Returns:
            是否可用
        """
        ...

    def get_elevator_by_floor(
        self,
        floor: int,
        campus_name: str
    ) -> Optional[Elevator]:
        """
        获取楼层对应的电梯

        Args:
            floor: 楼层号
            campus_name: 校园名称

        Returns:
            电梯对象
        """
        ...


class RobotStateProvider(Protocol):
    """
    机器人状态提供者接口

    定义获取机器人实时状态的接口。
    """

    def get_robot_position(
        self,
        robot_id: int
    ) -> Tuple[float, float, float]:
        """
        获取机器人位置

        Args:
            robot_id: 机器人ID

        Returns:
            位置坐标 (x, y, z)
        """
        ...

    def get_robot_battery(
        self,
        robot_id: int
    ) -> float:
        """
        获取机器人电量

        Args:
            robot_id: 机器人ID

        Returns:
            电量百分比
        """
        ...

    def is_robot_available(
        self,
        robot_id: int,
        current_time: float
    ) -> bool:
        """
        检查机器人是否可用

        Args:
            robot_id: 机器人ID
            current_time: 当前时间

        Returns:
            是否可用
        """
        ...