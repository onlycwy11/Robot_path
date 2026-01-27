"""
路径信息数据类模块

使用 dataclasses 提供类型安全、不可变性和更好的文档化。
保持 TypedDict 向后兼容。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Tuple, Optional, Dict, Any, TypedDict


# ==================== Dataclasses ====================

@dataclass
class PathSegment:
    """
    单段路径信息

    Attributes:
        nodes: 节点路径列表
        time: 该段耗时
        wait_time: 等待时间（电梯等待）
    """
    nodes: List[str] = field(default_factory=list)
    time: float = 0.0
    wait_time: Tuple[float, float] = (0.0, 0.0)


@dataclass
class ElevatorSegment:
    """
    电梯段信息

    Attributes:
        elevator_id: 电梯ID
        start_node: 进电梯节点
        end_node: 出电梯节点
        from_floor: 起始楼层
        to_floor: 目标楼层
        time: 电梯运行时间
        reservation_start: 预约开始时间
        reservation_end: 预约结束时间
    """
    elevator_id: Optional[str] = None
    start_node: Optional[str] = None
    end_node: Optional[str] = None
    from_floor: int = 0
    to_floor: int = 0
    time: float = 0.0
    reservation_start: float = 0.0
    reservation_end: float = 0.0

    @property
    def floor_diff(self) -> int:
        """楼层差"""
        return abs(self.to_floor - self.from_floor)


@dataclass
class FullPathInfo:
    """
    完整路径信息

    Attributes:
        first_segment: 第一段路径（取货/第一部电梯前）
        elevator_1: 第一部电梯信息（可选）
        transfer: 中间步行段（两部电梯间）
        elevator_2: 第二部电梯信息（可选）
        second_segment: 第二段路径（出电梯后）

        start_time: 任务起始时间
        total_time: 总耗时
        path_type: 路径类型 ("stair" | "elevator" | "mixed")
        status: 路径阶段 (0=pick, 1=deliver)
    """
    first_segment: PathSegment = field(default_factory=PathSegment)
    elevator_1: Optional[ElevatorSegment] = None
    transfer: float = 0.0  #两部电梯间步行时间
    elevator_2: Optional[ElevatorSegment] = None
    second_segment: PathSegment = field(default_factory=PathSegment)

    start_time: float = 0.0
    total_time: float = 0.0
    path_type: str = "stair"
    status: int = 0

    @property
    def full_path(self) -> List[str]:
        """获取完整节点路径"""
        path = self.first_segment.nodes.copy()

        if self.elevator_1:
            if self.elevator_1.start_node and self.elevator_1.start_node not in path:
                path.append(self.elevator_1.start_node)
            if self.elevator_1.end_node:
                path.append(self.elevator_1.end_node)

        if self.elevator_2:
            if self.elevator_2.start_node and self.elevator_2.start_node not in path:
                path.append(self.elevator_2.start_node)
            if self.elevator_2.end_node:
                path.append(self.elevator_2.end_node)

        path.extend(self.second_segment.nodes)
        return path

    @property
    def has_elevator(self) -> bool:
        """是否使用电梯"""
        return self.elevator_1 is not None or self.elevator_2 is not None

    def to_legacy_dict(self) -> PathInfo:
        """转换为旧版 TypedDict 格式"""
        return PathInfo(
            path=self.full_path,
            path_1=self.first_segment.nodes,
            path_2=self.second_segment.nodes,
            start_time=self.start_time,
            actual_time=self.total_time,
            part_time_1=self.first_segment.time,
            part_time_2=self.second_segment.time,
            wait_time_1=self.first_segment.wait_time[0],
            wait_time_2=self.first_segment.wait_time[1],

            before=self.first_segment.time,
            between_1=self.elevator_1.time if self.elevator_1 else 0.0,
            transfer=self.transfer,
            between=self.elevator_2.time if self.elevator_2 else 0.0,
            after=self.second_segment.time,

            start_e1=self.elevator_1.start_node if self.elevator_1 else None,
            end_e1=self.elevator_1.end_node if self.elevator_1 else None,
            start_e2=self.elevator_2.start_node if self.elevator_2 else None,
            end_e2=self.elevator_2.end_node if self.elevator_2 else None,

            eid_1=self.elevator_1.elevator_id if self.elevator_1 else None,
            eid_2=self.elevator_2.elevator_id if self.elevator_2 else None,

            type=self.path_type,
            status=self.status,
            elevator_stair=self.has_elevator,

            from_floor_1=self.elevator_1.from_floor if self.elevator_1 else 0,
            end_floor_1=self.elevator_1.to_floor if self.elevator_1 else 0,
            from_floor_2=self.elevator_2.from_floor if self.elevator_2 else 0,
            end_floor_2=self.elevator_2.to_floor if self.elevator_2 else 0,

            elev_start_1=self.elevator_1.reservation_start if self.elevator_1 else 0.0,
            elev_end_1=self.elevator_1.reservation_end if self.elevator_1 else 0.0,
            elev_start_2=self.elevator_2.reservation_start if self.elevator_2 else 0.0,
            elev_end_2=self.elevator_2.reservation_end if self.elevator_2 else 0.0,
        )


@dataclass
class ElevatorUsageRecord:
    """
    电梯使用记录

    Attributes:
        elevator_id: 电梯ID
        robot_id: 使用机器人ID
        time_window: 使用时间窗口 (start, end)
        status: 路径阶段 (0=pick, 1=deliver)
        part: 电梯段 (1=第一部, 2=第二部)
    """
    elevator_id: str = ""
    robot_id: int = 0
    time_window: Tuple[float, float] = (0.0, 0.0)
    status: int = 0
    part: int = 1


@dataclass
class ConflictInfo:
    """
    冲突信息

    Attributes:
        elevator_id: 电梯ID
        usage_1: 第一个使用记录
        usage_2: 第二个使用记录
        overlap_time: 重叠时间
        start_time: 冲突开始时间
    """
    elevator_id: str = ""
    usage_1: Optional[ElevatorUsageRecord] = None
    usage_2: Optional[ElevatorUsageRecord] = None
    overlap_time: float = 0.0
    start_time: float = 0.0


@dataclass
class TaskAssignment:
    """
    任务分配信息

    Attributes:
        task_id: 任务ID
        robot_id: 分配机器人ID
        pick_path: 取货路径信息
        deliver_path: 送货路径信息
        release_time: 任务发布时间
        start_time: 任务开始时间
        end_time: 任务结束时间
    """
    task_id: int = 0
    robot_id: int = 0
    pick_path: Optional[FullPathInfo] = None
    deliver_path: Optional[FullPathInfo] = None
    release_time: float = 0.0
    start_time: float = 0.0
    end_time: float = 0.0


# ==================== TypedDict (向后兼容) ====================

class PathSegments(TypedDict, total=False):
    """路径分段信息（旧版兼容）"""
    before: float
    between_1: float
    transfer: float
    between: float
    after: float


class PathInfo(TypedDict, total=False):
    """路径信息完整结构（旧版兼容）"""
    path: List[str]
    path_1: List[str]
    path_2: List[str]
    start_time: float
    actual_time: float
    part_time_1: float
    part_time_2: float
    wait_time_1: float
    wait_time_2: float

    before: float
    between_1: float
    transfer: float
    between: float
    after: float

    start_e1: Optional[str]
    end_e1: Optional[str]
    start_e2: Optional[str]
    end_e2: Optional[str]

    eid_1: Optional[str]
    eid_2: Optional[str]

    type: str
    status: int
    elevator_stair: bool

    from_floor_1: int
    end_floor_1: int
    from_floor_2: int
    end_floor_2: int

    elev_start_1: float
    elev_end_1: float
    elev_start_2: float
    elev_end_2: float


class ElevatorUsage(TypedDict):
    """电梯使用记录（旧版兼容）"""
    assignment: Dict[str, Any]
    status: int
    part: int
    time_window: Tuple[float, float]
    robot_id: int


class Conflict(TypedDict):
    """冲突信息（旧版兼容）"""
    elevator_id: str
    usage1: ElevatorUsage
    usage2: ElevatorUsage
    overlap_time: float
    start_time: float


class Assignment(TypedDict, total=False):
    """任务分配信息（旧版兼容）"""
    task: Any
    robot_id: int
    pick_path_info: PathInfo
    deliver_path_info: PathInfo
    path_results: Dict[str, PathInfo]
    release_time: float
    start_time: float
    end_time: float
    selected: List[str]


class AlternativeRoute(TypedDict, total=False):
    """替代路径信息（旧版兼容）"""
    type: str
    elevator_id: Optional[str]
    path_info: PathInfo
    part: Optional[int]
    status: int
    part_time_1: Optional[float]
    part_time_2: float
    original_actual_time: float
    description: str
    from_cache: bool
    wait_time: Optional[float]


# ==================== 转换函数 ====================

def from_legacy_path_info(d: PathInfo) -> FullPathInfo:
    """从旧版 TypedDict 转换为 FullPathInfo dataclass"""
    first_seg = PathSegment(
        nodes=d.get('path_1', []),
        time=d.get('part_time_1', 0.0),
        wait_time=(d.get('wait_time_1', 0.0), d.get('wait_time_2', 0.0))
    )

    second_seg = PathSegment(
        nodes=d.get('path_2', []),
        time=d.get('part_time_2', 0.0),
        wait_time=(0.0, 0.0)
    )

    elev_1 = None
    if d.get('eid_1'):
        elev_1 = ElevatorSegment(
            elevator_id=d['eid_1'],
            start_node=d.get('start_e1'),
            end_node=d.get('end_e1'),
            from_floor=d.get('from_floor_1', 0),
            to_floor=d.get('end_floor_1', 0),
            time=d.get('between_1', 0.0),
            reservation_start=d.get('elev_start_1', 0.0),
            reservation_end=d.get('elev_end_1', 0.0)
        )

    elev_2 = None
    if d.get('eid_2'):
        elev_2 = ElevatorSegment(
            elevator_id=d['eid_2'],
            start_node=d.get('start_e2'),
            end_node=d.get('end_e2'),
            from_floor=d.get('from_floor_2', 0),
            to_floor=d.get('end_floor_2', 0),
            time=d.get('between', 0.0),
            reservation_start=d.get('elev_start_2', 0.0),
            reservation_end=d.get('elev_end_2', 0.0)
        )

    return FullPathInfo(
        first_segment=first_seg,
        elevator_1=elev_1,
        transfer=d.get('transfer', 0.0),
        elevator_2=elev_2,
        second_segment=second_seg,
        start_time=d.get('start_time', 0.0),
        total_time=d.get('actual_time', 0.0),
        path_type=d.get('type', 'stair'),
        status=d.get('status', 0)
    )