"""
Scheduler 模块单元测试

测试 BatchScheduler、ConflictResolver 功能。
"""

import pytest
from typing import Dict, List

from src.schedulers.conflict_resolver import ConflictResolver
from src.models.elevator import Elevator, init_elevators
from src.utils.config import DEFAULT_CONFIG


class TestConflictResolver:
    """冲突解决器测试"""

    def setup_method(self):
        # 使用实际的电梯对象初始化
        elevators = init_elevators(["1_E1", "1_E2"])
        self.resolver = ConflictResolver(elevators, DEFAULT_CONFIG)

    def test_initialization(self):
        """测试初始化"""
        assert self.resolver._simulated_reservations == {}

    def test_clear_reservations(self):
        """测试清空预约"""
        # 添加一些预约数据
        self.resolver._simulated_reservations["1_E1"] = [
            {"robot_id": 1, "start_time": 0.0, "end_time": 10.0}
        ]

        self.resolver.clear_reservations()

        assert self.resolver._simulated_reservations == {}

    def test_get_reservations(self):
        """测试获取预约状态"""
        self.resolver._simulated_reservations["1_E1"] = [
            {"robot_id": 1, "start_time": 0.0, "end_time": 10.0}
        ]

        reservations = self.resolver.get_reservations()

        assert "1_E1" in reservations


class TestSchedulerUtils:
    """调度器辅助功能测试"""

    def test_path_time_calculation(self):
        """测试路径时间计算"""
        # 简单的时间计算逻辑测试
        # 平地速度 150 cm/s
        from src.utils.constants import SPEED_LAND_CM

        distance = 1500  # 15米
        expected_time = distance / SPEED_LAND_CM  # 10秒

        assert expected_time == 10.0

    def test_elevator_time_calculation(self):
        """测试电梯时间计算"""
        from src.utils.config import calculate_elevator_time_need

        # 公式: 开门 + 关门 + 运行 + 开门 = 1.5 + 1.5 + 1.75*n + 1.5
        # n=2: 1.5 + 1.5 + 3.5 + 1.5 = 8.0
        time = calculate_elevator_time_need(2)
        assert time == 8.0

    def test_elevator_time_table(self):
        """测试电梯时间预计算表"""
        from src.utils.config import ELEVATOR_TIME_TABLE, get_elevator_time_from_table, calculate_elevator_time_need

        # 检查表中有预计算值
        assert 1 in ELEVATOR_TIME_TABLE
        assert ELEVATOR_TIME_TABLE[1] == 6.25

        # 使用表查询
        assert get_elevator_time_from_table(3) == 9.75

        # 超出表范围
        assert get_elevator_time_from_table(15) == calculate_elevator_time_need(15)


# pytest 配置
if __name__ == "__main__":
    pytest.main([__file__, "-v"])