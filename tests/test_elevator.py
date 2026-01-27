"""
Elevator 模块单元测试

测试 Elevator、Stair 类功能。
"""

import pytest

from src.models.elevator import Elevator, Stair, ElevatorScheduleEntry, init_elevators
from src.utils.config import DEFAULT_CONFIG


class TestElevator:
    """电梯类测试"""

    def test_elevator_initialization(self):
        """测试电梯初始化"""
        elevator = Elevator(
            eid=1,
            bldg_num=1,
            local_id="E1",
            config=DEFAULT_CONFIG
        )

        assert elevator.id == 1
        assert elevator.bldg_num == 1
        assert elevator.local_id == "E1"

    def test_elevator_schedule_entry(self):
        """测试电梯预约条目"""
        entry = ElevatorScheduleEntry(
            start_time=0.0,
            end_time=10.0,
            from_floor=1,
            to_floor=4,
            robot_id=1
        )

        assert entry.robot_id == 1
        assert entry.start_time == 0.0
        assert entry.end_time == 10.0
        assert entry.from_floor == 1
        assert entry.to_floor == 4

    def test_elevator_reserve(self):
        """测试电梯预约"""
        elevator = Elevator(eid=1, bldg_num=1, local_id="E1")

        elevator.reserve(
            start_time=0.0,
            end_time=10.0,
            from_floor=1,
            to_floor=4,
            robot_id=1
        )

        assert len(elevator.schedule) == 1
        assert elevator.schedule[0].robot_id == 1

    def test_elevator_get_current_floor(self):
        """测试获取当前楼层"""
        elevator = Elevator(eid=1, bldg_num=1, local_id="E1", initial_floor=1)

        # 无预约时返回初始楼层
        assert elevator.get_current_floor(0.0) == 1

        # 添加预约
        elevator.reserve(0.0, 10.0, 1, 4, 1)

        # 预约期间返回目标楼层
        assert elevator.get_current_floor(5.0) == 4


class TestStair:
    """楼梯类测试"""

    def test_stair_initialization(self):
        """测试楼梯初始化"""
        stair = Stair(stair_id="Stair1", building_num=1, floor_num=4)

        assert stair.id == "Stair1"
        assert stair.building_num == 1
        assert stair.floor_num == 4

    def test_stair_reserve(self):
        """测试楼梯预约"""
        stair = Stair(stair_id="Stair1", building_num=1, floor_num=4)

        stair.reserve(start_time=0.0, duration=10.0, robot_id=1, task_id=1)

        assert len(stair.usage_schedule) == 1


class TestInitElevators:
    """电梯初始化测试"""

    def test_init_elevators(self):
        """测试电梯初始化函数"""
        elevator_names = ["1_E1", "1_E2", "2_E1"]
        elevators = init_elevators(elevator_names)

        assert len(elevators) == 3
        assert "1_E1" in elevators
        assert "1_E2" in elevators
        assert "2_E1" in elevators


# pytest 配置
if __name__ == "__main__":
    pytest.main([__file__, "-v"])