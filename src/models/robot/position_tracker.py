"""
机器人位置追踪器

管理机器人实时位置计算和状态更新。
"""

from typing import List, Tuple, Optional

from src.utils.config import SystemConfig, DEFAULT_CONFIG
from src.utils.logger import robot_logger


class RobotPositionTracker:
    """
    机器人位置追踪器

    根据任务路径计算机器人实时位置，支持电梯等待时间。
    """

    def __init__(
        self,
        robot_id: int,
        campus_name: str,
        config: SystemConfig = DEFAULT_CONFIG
    ):
        self.robot_id = robot_id
        self.campus_name = campus_name
        self.config = config

        # 当前位置（坐标）
        self.current_position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.current_node: Optional[str] = None

        # 路径信息（由外部设置）
        self.path: List[str] = []
        self.path1: List[str] = []
        self.path2: List[str] = []
        self.path_start_time: Optional[float] = None
        self.path_total_time: float = 0.0
        self.pick_time: float = 0.0
        self.deliver_time: float = 0.0
        self.wait_pair_1: Tuple[float, float] = (0.0, 0.0)
        self.wait_pair_2: Tuple[float, float] = (0.0, 0.0)
        self.running_time: float = 0.0

    def set_path(
        self,
        path: List[str],
        path1: List[str],
        path2: List[str],
        start_time: float,
        total_time: float,
        pick_time: float,
        deliver_time: float,
        wait_pair_1: Tuple[float, float],
        wait_pair_2: Tuple[float, float]
    ) -> None:
        """设置当前路径信息"""
        self.path = path
        self.path1 = path1
        self.path2 = path2
        self.path_start_time = start_time
        self.path_total_time = total_time
        self.pick_time = pick_time
        self.deliver_time = deliver_time
        self.wait_pair_1 = wait_pair_1
        self.wait_pair_2 = wait_pair_2

    def clear_path(self) -> None:
        """清空路径信息"""
        self.path = []
        self.path1 = []
        self.path2 = []
        self.path_start_time = None
        self.path_total_time = 0.0
        self.pick_time = 0.0
        self.deliver_time = 0.0
        self.running_time = 0.0

    def update_position(self, current_time: float) -> None:
        """
        更新当前位置

        Args:
            current_time: 当前系统时间
        """
        if not self.path:
            return

        try:
            from src.core.node import get_xyz_from_path_and_time_with_elevator_wait

            if current_time < self.pick_time:
                x, y, z = get_xyz_from_path_and_time_with_elevator_wait(
                    path_list=self.path1,
                    t=current_time,
                    wait_time_1=self.wait_pair_1[0],
                    wait_time_2=self.wait_pair_1[1],
                    campus_name=self.campus_name
                )
            else:
                x, y, z = get_xyz_from_path_and_time_with_elevator_wait(
                    path_list=self.path2,
                    t=current_time - self.pick_time,
                    wait_time_1=self.wait_pair_2[0],
                    wait_time_2=self.wait_pair_2[1],
                    campus_name=self.campus_name
                )

            self.current_position = (x, y, z)

        except Exception as e:
            robot_logger.warning(f"Position update failed: {e}")

    def set_initial_position(self, node: str) -> None:
        """设置初始位置"""
        try:
            from src.core.node import get_coordinates_from_node
            self.current_node = node
            self.current_position = get_coordinates_from_node(node, self.campus_name)
        except Exception as e:
            robot_logger.warning(f"Failed to get coordinates for node {node}: {e}")