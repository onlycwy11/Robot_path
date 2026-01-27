from __future__ import annotations

"""
电梯和楼梯数据模型

定义电梯类、楼梯类及其调度管理功能。
"""

from dataclasses import dataclass, field
from typing import List, Tuple, Dict, Optional, DefaultDict
from collections import defaultdict
import copy

from src.utils.config import DEFAULT_CONFIG, SystemConfig
from src.utils.logger import robot_logger


@dataclass
class ElevatorScheduleEntry:
    """
    电梯调度条目

    Attributes:
        start_time: 预约开始时间
        end_time: 预约结束时间
        from_floor: 起始楼层
        to_floor: 目标楼层
        robot_id: 使用机器人ID
    """
    start_time: float
    end_time: float
    from_floor: int
    to_floor: int
    robot_id: int


class Elevator:
    """
    电梯类

    管理电梯状态和调度表。

    Attributes:
        id: 电梯编号
        bldg_num: 所属大楼编号
        local_id: 本地标识（如 "E1", "E2")
        initial_floor: 初始楼层
        schedule: 调度表列表
        config: 系统配置
    """

    def __init__(
        self,
        eid: int,
        bldg_num: int,
        local_id: str,
        initial_floor: int = 1,
        config: SystemConfig = DEFAULT_CONFIG
    ):
        self.id = eid
        self.bldg_num = bldg_num
        self.local_id = local_id
        self.initial_floor = initial_floor
        self.config = config

        # 电梯调度表：开始时间、结束时间、起始楼层、目标楼层、使用电梯的机器人ID
        self.schedule: List[ElevatorScheduleEntry] = []

    def get_current_floor(self, current_time: float) -> int:
        """
        根据当前时间和调度表，计算电梯的实际楼层

        Args:
            current_time: 当前时间（秒）

        Returns:
            当前楼层号
        """
        if not self.schedule:
            return self.initial_floor

        # 按开始时间排序
        sorted_sched = sorted(self.schedule, key=lambda x: x.start_time)

        for i, entry in enumerate(sorted_sched):
            if current_time < entry.start_time:
                # 在当前预约开始前
                if i == 0:
                    return self.initial_floor
                else:
                    # 返回上一个预约结束后的位置
                    prev_entry = sorted_sched[i - 1]
                    return prev_entry.to_floor

            elif entry.start_time <= current_time <= entry.end_time:
                # 在预约执行期间，返回目标楼层
                return entry.to_floor

        # 在所有预约之后
        last_entry = sorted_sched[-1]
        return last_entry.to_floor

    def check_reserve(
        self,
        current_time: float,
        before: float,
        between: float,
        from_floor: int,
        elevator_position_at_start: int = 0
    ) -> Tuple[float, float, float]:
        """
        检查是否可以正常预约电梯

        Args:
            current_time: 当前时间
            before: 到达电梯前的行走时间
            between: 电梯运行时间
            from_floor: 起始楼层
            elevator_position_at_start: 电梯起始位置（可选）

        Returns:
            (预约开始时间, 预约结束时间, 电梯就绪时间)
        """
        # 电梯当前所在楼层
        if not elevator_position_at_start:
            elevator_position_at_start = self.get_current_floor(current_time)

        # 电梯准备就绪所需的移动时间
        travel_to_start = abs(from_floor - elevator_position_at_start) * self.config.elevator_per_floor

        # 本次电梯预约的开始时间
        elev_start = current_time + before - travel_to_start if before > travel_to_start else current_time
        elev_ready = current_time + travel_to_start

        # 初始化等待时间
        wait_time = 0.0
        delta = 0.0
        new_current_time = current_time

        # 检查电梯调度表中的冲突
        for entry in self.schedule:
            # 无冲突：电梯停止时间早于预约开始，或启动时间晚于预约结束
            if not (elev_start + travel_to_start + between <= entry.start_time or elev_start >= entry.end_time):
                # 有冲突，需要等待
                wait_time = max(wait_time, entry.end_time - elev_start)
                delta = before - (entry.end_time - current_time)
                new_current_time = max(new_current_time, entry.end_time)

        if wait_time:
            # 预约有冲突，需至下一个时间段重新预约
            before_new = delta if delta > 0 else 0
            elev_start, elev_end, elev_ready = self.check_reserve(
                new_current_time, before_new, between, from_floor
            )
            return elev_start, elev_end, elev_ready

        elev_end = elev_start + wait_time + travel_to_start + between
        return elev_start, elev_end, elev_ready

    def can_reserve_early_move(
        self,
        t_now: float,
        current_floor: int,
        target_floor: int
    ) -> Tuple[bool, float]:
        """
        检查是否可以提前调度电梯到目标楼层（不干扰现有预约）

        Args:
            t_now: 当前时间
            current_floor: 当前楼层
            target_floor: 目标楼层

        Returns:
            (是否可提前移动, 移动时间)
        """
        # 找到下一个任务
        next_task = None
        previous_task = None
        for i, entry in enumerate(self.schedule):
            if entry.start_time > t_now:
                next_task = entry
                if i > 0:
                    previous_task = self.schedule[i - 1]
                break

        # 计算空闲时段
        free_start = previous_task.end_time if previous_task and t_now < previous_task.end_time else t_now
        free_end = next_task.start_time if next_task else float('inf')

        # 估算移动时间
        move_time = abs(target_floor - current_floor) * self.config.elevator_per_floor

        # 检查是否能在空闲时段完成移动
        if free_start + move_time < free_end:
            return True, move_time
        else:
            # 若无法完成，则等待下一个空闲时间段
            current_floor = self.get_current_floor(next_task.end_time)
            _, move_time = self.can_reserve_early_move(next_task.end_time, current_floor, target_floor)
            return False, move_time + next_task.end_time - t_now

    def reserve(
        self,
        start_time: float,
        end_time: float,
        from_floor: int,
        to_floor: int,
        robot_id: int
    ):
        """
        预约电梯使用

        Args:
            start_time: 预约开始时间
            end_time: 预约结束时间
            from_floor: 起始楼层
            to_floor: 目标楼层
            robot_id: 使用机器人ID
        """
        entry = ElevatorScheduleEntry(
            start_time=start_time,
            end_time=end_time,
            from_floor=from_floor,
            to_floor=to_floor,
            robot_id=robot_id
        )
        self.schedule.append(entry)
        self.schedule.sort(key=lambda x: x.start_time)
        robot_logger.info(
            f"[Elevator {self.id} Reserved] R{robot_id}: "
            f"{from_floor}->{to_floor}, {start_time:.2f}s - {end_time:.2f}s"
        )

    def clear_schedule(self):
        """清空调度表"""
        self.schedule.clear()

    def cancel_reservation(self, start_time: float) -> bool:
        """
        取消电梯预约

        Args:
            start_time: 预约开始时间

        Returns:
            是否成功取消
        """
        for i, entry in enumerate(self.schedule):
            if entry.start_time == start_time:
                self.schedule.pop(i)
                robot_logger.info(
                    f"[Elevator {self.id} Cancelled] R{entry.robot_id}: "
                    f"{entry.from_floor}->{entry.to_floor}, {start_time:.2f}s"
                )
                return True
        return False

    def cancel_robot_reservations(self, robot_id: int) -> int:
        """
        取消指定机器人的所有预约

        Args:
            robot_id: 机器人ID

        Returns:
            取消的预约数量
        """
        cancelled = 0
        new_schedule = []

        for entry in self.schedule:
            if entry.robot_id == robot_id:
                cancelled += 1
                robot_logger.info(
                    f"[Elevator {self.id} Cancelled] R{entry.robot_id}: "
                    f"{entry.from_floor}->{entry.to_floor}"
                )
            else:
                new_schedule.append(entry)

        self.schedule = new_schedule
        return cancelled

    def get_schedule(self) -> List[ElevatorScheduleEntry]:
        """获取调度表"""
        return self.schedule.copy()

    def get_schedule_summary(self) -> str:
        """获取调度表摘要"""
        if not self.schedule:
            return f"Elevator {self.id}: No reservations"
        lines = [f"Elevator {self.id} Schedule:"]
        for entry in self.schedule:
            lines.append(
                f"  R{entry.robot_id}: {entry.from_floor}→{entry.to_floor}, "
                f"{entry.start_time:.2f}s-{entry.end_time:.2f}s"
            )
        return "\n".join(lines)


class Stair:
    """
    楼梯类

    用于检测和管理楼梯超车、会车问题。

    Attributes:
        id: 楼梯标识
        building_num: 所属大楼编号
        floor_num: 楼层编号
        usage_schedule: 使用调度表
    """

    def __init__(self, stair_id: str, building_num: int, floor_num: int):
        self.id = stair_id
        self.building_num = building_num
        self.floor_num = floor_num
        # (start_time, end_time, robot_id, direction)
        self.usage_schedule: List[Tuple[float, float, int, int]] = []

    def reserve(
        self,
        start_time: float,
        duration: float,
        robot_id: int,
        task_id: int
    ):
        """
        预约楼梯使用

        Args:
            start_time: 开始时间
            duration: 使用时长
            robot_id: 机器人ID
            task_id: 任务ID
        """
        end_time = start_time + duration
        self.usage_schedule.append((start_time, end_time, robot_id, task_id))
        self.usage_schedule.sort(key=lambda x: x[0])
        robot_logger.info(
            f"[Stair {self.id} Reserved] R{robot_id} (Task{task_id}): "
            f"{start_time:.2f}s - {end_time:.2f}s"
        )

    def check_availability(
        self,
        desired_start: float,
        duration: float
    ) -> Tuple[bool, float, float]:
        """
        检查楼梯可用性

        Args:
            desired_start: 希望的开始时间
            duration: 使用时长

        Returns:
            (是否可用, 实际开始时间, 等待时间)
        """
        desired_end = desired_start + duration

        if not self.usage_schedule:
            return True, desired_start, 0.0

        # 检查当前时段是否有冲突
        actual_start = desired_start
        for scheduled_start, scheduled_end, robot_id, task_id in self.usage_schedule:
            if actual_start + duration <= scheduled_start:
                # 找到可用时间段
                break
            elif actual_start < scheduled_end:
                # 有冲突，需要等待
                actual_start = scheduled_end

        wait_time = max(0, actual_start - desired_start)
        return True, actual_start, wait_time

    def copy(self) -> Stair:
        """创建楼梯的深拷贝"""
        new_stair = Stair(self.id, self.building_num, self.floor_num)
        new_stair.usage_schedule = copy.deepcopy(self.usage_schedule)
        return new_stair


def init_elevators(elevator_names: List[str], config: SystemConfig = DEFAULT_CONFIG) -> Dict[str, Elevator]:
    """
    初始化电梯字典

    Args:
        elevator_names: 电梯名称列表（格式: "{building}_E{number}"）
        config: 系统配置

    Returns:
        电梯名称 -> Elevator 对象的字典
    """
    elevators = {}
    index = 0
    for elevator_name in elevator_names:
        index += 1
        parts = elevator_name.split("_")
        if len(parts) == 2:
            number_part = parts[0]
            string_part = parts[1]

            try:
                number_part = int(number_part)
                elevators[elevator_name] = Elevator(
                    eid=index,
                    bldg_num=number_part,
                    local_id=string_part,
                    config=config
                )
            except ValueError:
                robot_logger.error(f"'{number_part}' is not a valid number")
        else:
            robot_logger.error(f"Invalid format '{elevator_name}', expected 'number_string'")

    return elevators