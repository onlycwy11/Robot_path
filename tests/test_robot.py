"""
Robot 模块单元测试

测试 Robot 类、TaskManager、PositionTracker 功能。
"""

import pytest
import time
from typing import Dict

from src.models.robot import Robot, RobotTaskInfo
from src.models.robot.task_manager import RobotTaskManager
from src.models.robot.position_tracker import RobotPositionTracker
from src.utils.constants import RobotType, RobotStatus
from src.utils.config import DEFAULT_CONFIG


class TestRobotTaskManager:
    """任务管理器测试"""

    def setup_method(self):
        self.manager = RobotTaskManager(robot_id=1)

    def test_add_task(self):
        """测试添加任务"""
        task_info = self.manager.add_task(
            task_id=1,
            start_time=0.0,
            finish_time=10.0,
            path=["A", "B", "C"],
            path1=["A", "B"],
            path2=["B", "C"],
            pick_time=5.0,
            deliver_time=5.0,
            wait_pair_1=(1.0, 2.0),
            wait_pair_2=(0.0, 0.0)
        )

        assert self.manager.get_task_count() == 1
        assert task_info.task_id == 1
        assert task_info.start_time == 0.0

    def test_remove_task(self):
        """测试移除任务"""
        self.manager.add_task(
            task_id=1,
            start_time=0.0,
            finish_time=10.0,
            path=["A", "B"],
            path1=["A"],
            path2=["B"],
            pick_time=5.0,
            deliver_time=5.0,
            wait_pair_1=(0.0, 0.0),
            wait_pair_2=(0.0, 0.0)
        )

        removed = self.manager.remove_task(1)

        assert removed is not None
        assert removed.task_id == 1
        assert self.manager.get_task_count() == 0

    def test_remove_nonexistent_task(self):
        """测试移除不存在的任务"""
        removed = self.manager.remove_task(999)

        assert removed is None

    def test_get_sorted_tasks(self):
        """测试按时间排序任务"""
        self.manager.add_task(
            task_id=3, start_time=20.0, finish_time=30.0,
            path=[], path1=[], path2=[], pick_time=0, deliver_time=0,
            wait_pair_1=(0, 0), wait_pair_2=(0, 0)
        )
        self.manager.add_task(
            task_id=1, start_time=0.0, finish_time=10.0,
            path=[], path1=[], path2=[], pick_time=0, deliver_time=0,
            wait_pair_1=(0, 0), wait_pair_2=(0, 0)
        )
        self.manager.add_task(
            task_id=2, start_time=10.0, finish_time=20.0,
            path=[], path1=[], path2=[], pick_time=0, deliver_time=0,
            wait_pair_1=(0, 0), wait_pair_2=(0, 0)
        )

        sorted_tasks = self.manager.get_sorted_tasks()
        task_ids = list(sorted_tasks.keys())

        assert task_ids == [1, 2, 3]  # 按开始时间排序

    def test_get_active_task(self):
        """测试获取当前活动任务"""
        self.manager.add_task(
            task_id=1, start_time=5.0, finish_time=15.0,
            path=[], path1=[], path2=[], pick_time=0, deliver_time=0,
            wait_pair_1=(0, 0), wait_pair_2=(0, 0)
        )

        # 在任务时间范围内
        active = self.manager.get_active_task(10.0)
        assert active is not None
        assert active.task_id == 1

        # 在任务时间范围外
        active = self.manager.get_active_task(20.0)
        assert active is None

    def test_clear_tasks(self):
        """测试清空任务"""
        self.manager.add_task(
            task_id=1, start_time=0.0, finish_time=10.0,
            path=[], path1=[], path2=[], pick_time=0, deliver_time=0,
            wait_pair_1=(0, 0), wait_pair_2=(0, 0)
        )
        self.manager.add_task(
            task_id=2, start_time=10.0, finish_time=20.0,
            path=[], path1=[], path2=[], pick_time=0, deliver_time=0,
            wait_pair_1=(0, 0), wait_pair_2=(0, 0)
        )

        self.manager.clear_tasks()

        assert self.manager.get_task_count() == 0

    def test_has_task(self):
        """测试检查任务存在"""
        self.manager.add_task(
            task_id=1, start_time=0.0, finish_time=10.0,
            path=[], path1=[], path2=[], pick_time=0, deliver_time=0,
            wait_pair_1=(0, 0), wait_pair_2=(0, 0)
        )

        assert self.manager.has_task(1) == True
        assert self.manager.has_task(999) == False


class TestRobotPositionTracker:
    """位置追踪器测试"""

    def setup_method(self):
        self.tracker = RobotPositionTracker(
            robot_id=1,
            campus_name="test_campus",
            config=DEFAULT_CONFIG
        )

    def test_initial_state(self):
        """测试初始状态"""
        assert self.tracker.current_position == (0.0, 0.0, 0.0)
        assert self.tracker.current_node is None
        assert self.tracker.path == []

    def test_set_path(self):
        """测试设置路径"""
        self.tracker.set_path(
            path=["A", "B", "C"],
            path1=["A", "B"],
            path2=["B", "C"],
            start_time=0.0,
            total_time=10.0,
            pick_time=5.0,
            deliver_time=5.0,
            wait_pair_1=(1.0, 2.0),
            wait_pair_2=(0.0, 0.0)
        )

        assert self.tracker.path == ["A", "B", "C"]
        assert self.tracker.path_start_time == 0.0

    def test_clear_path(self):
        """测试清空路径"""
        self.tracker.set_path(
            path=["A", "B"], path1=["A"], path2=["B"],
            start_time=0.0, total_time=10.0,
            pick_time=5.0, deliver_time=5.0,
            wait_pair_1=(0, 0), wait_pair_2=(0, 0)
        )

        self.tracker.clear_path()

        assert self.tracker.path == []
        assert self.tracker.path_start_time is None


class TestRobot:
    """Robot 类测试"""

    def test_robot_initialization(self):
        """测试 Robot 初始化"""
        robot = Robot(
            rid=1,
            skill="dog",
            position="4_1_p1",
            campus_name="test_campus",
            enable_mqtt_charge_updates=False
        )

        assert robot.id == 1
        assert robot.skill == "dog"
        assert robot.position == "4_1_p1"
        assert robot.charge == 100.0

    def test_robot_get_type(self):
        """测试获取机器人类型"""
        dog_robot = Robot(rid=1, skill="dog", position="A")
        human_robot = Robot(rid=2, skill="human", position="B")

        assert dog_robot.get_type() == RobotType.DOG
        assert human_robot.get_type() == RobotType.HUMAN

    def test_robot_is_available(self):
        """测试机器人可用性"""
        robot = Robot(rid=1, skill="dog", position="A")

        # 电量充足，不在充电
        robot.charge = 80.0
        robot.is_charging = False
        assert robot.is_available() == True

        # 电量不足
        robot.charge = 10.0
        assert robot.is_available() == False

        # 正在充电
        robot.charge = 80.0
        robot.is_charging = True
        assert robot.is_available() == False

    def test_robot_repr(self):
        """测试 Robot 字符串表示"""
        robot = Robot(rid=1, skill="dog", position="A")

        repr_str = repr(robot)
        assert "Robot" in repr_str
        assert "id=1" in repr_str
        assert "dog" in repr_str

    def test_robot_stop(self):
        """测试 Robot 停止"""
        robot = Robot(rid=1, skill="dog", position="A")

        robot.stop()

        # 验证停止后资源已清理
        assert robot._running == False


# pytest 配置
if __name__ == "__main__":
    pytest.main([__file__, "-v"])