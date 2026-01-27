"""
自定义异常类模块

定义系统中使用的自定义异常，提供明确的错误信息。
"""

from typing import Optional, Tuple, List, Any


class SchedulerError(Exception):
    """调度器基础异常"""

    def __init__(self, message: str, details: Optional[dict] = None):
        self.message = message
        self.details = details or {}
        super().__init__(message)


# 路径相关异常
class PathNotFoundError(SchedulerError):
    """路径不存在异常"""

    def __init__(self, start: str, target: str):
        super().__init__(
            f"No valid path found from '{start}' to '{target}'",
            {"start": start, "target": target}
        )
        self.start = start
        self.target = target


class NodeNotFoundError(SchedulerError):
    """节点不存在异常"""

    def __init__(self, node: str, campus: str = None):
        campus_info = f" in campus '{campus}'" if campus else ""
        super().__init__(f"Node '{node}' not found{campus_info}", {"node": node, "campus": campus})
        self.node = node
        self.campus = campus


class InvalidPositionError(SchedulerError):
    """无效位置格式异常"""

    def __init__(self, position: str, reason: str = None):
        message = f"Invalid position format: '{position}'"
        if reason:
            message += f" ({reason})"
        super().__init__(message, {"position": position, "reason": reason})
        self.position = position
        self.reason = reason


# 电梯相关异常
class ElevatorUnavailableError(SchedulerError):
    """电梯不可用异常"""

    def __init__(self, elevator_id: str, time_window: Tuple[float, float]):
        start, end = time_window
        super().__init__(
            f"Elevator '{elevator_id}' unavailable during {start:.2f}s - {end:.2f}s",
            {"elevator_id": elevator_id, "time_window": time_window}
        )
        self.elevator_id = elevator_id
        self.time_window = time_window


class ElevatorConflictError(SchedulerError):
    """电梯冲突异常"""

    def __init__(self, elevator_id: str, conflicting_robots: List[int], overlap_time: float):
        super().__init__(
            f"Elevator '{elevator_id}' conflict: robots {conflicting_robots} overlap for {overlap_time:.2f}s",
            {"elevator_id": elevator_id, "conflicting_robots": conflicting_robots, "overlap_time": overlap_time}
        )
        self.elevator_id = elevator_id
        self.conflicting_robots = conflicting_robots
        self.overlap_time = overlap_time


class ConflictResolutionError(SchedulerError):
    """冲突解决失败异常"""

    def __init__(self, elevator_id: str, iterations: int):
        super().__init__(
            f"Failed to resolve elevator '{elevator_id}' conflict after {iterations} iterations",
            {"elevator_id": elevator_id, "iterations": iterations}
        )
        self.elevator_id = elevator_id
        self.iterations = iterations


# 机器人相关异常
class NoFeasibleRobotError(SchedulerError):
    """无可行机器人异常"""

    def __init__(self, task_id: int, skill: str):
        super().__init__(
            f"No feasible robot found for task {task_id} (skill: '{skill}')",
            {"task_id": task_id, "skill": skill}
        )
        self.task_id = task_id
        self.skill = skill


class RobotChargeLowError(SchedulerError):
    """机器人电量不足异常"""

    def __init__(self, robot_id: int, charge_level: float, threshold: float):
        super().__init__(
            f"Robot {robot_id} charge ({charge_level:.1f}%) below threshold ({threshold:.1f}%)",
            {"robot_id": robot_id, "charge_level": charge_level, "threshold": threshold}
        )
        self.robot_id = robot_id
        self.charge_level = charge_level
        self.threshold = threshold


class RobotNotFoundError(SchedulerError):
    """机器人不存在异常"""

    def __init__(self, robot_id: int):
        super().__init__(f"Robot with ID {robot_id} not found", {"robot_id": robot_id})
        self.robot_id = robot_id


class RobotBusyError(SchedulerError):
    """机器人忙碌异常"""

    def __init__(self, robot_id: int, available_time: float):
        super().__init__(f"Robot {robot_id} is busy until {available_time:.2f}s", {"robot_id": robot_id, "available_time": available_time})
        self.robot_id = robot_id
        self.available_time = available_time


# 任务相关异常
class TaskNotFoundError(SchedulerError):
    """任务不存在异常"""

    def __init__(self, task_id: int, robot_id: int = None):
        if robot_id:
            message = f"Task {task_id} not found on robot {robot_id}"
        else:
            message = f"Task {task_id} not found"
        super().__init__(message, {"task_id": task_id, "robot_id": robot_id})
        self.task_id = task_id
        self.robot_id = robot_id


class TaskAlreadyCompletedError(SchedulerError):
    """任务已完成异常"""

    def __init__(self, task_id: int, finish_time: float):
        super().__init__(f"Task {task_id} already completed at {finish_time:.2f}s", {"task_id": task_id, "finish_time": finish_time})
        self.task_id = task_id
        self.finish_time = finish_time


class InvalidTaskInputError(SchedulerError):
    """无效任务输入异常"""

    def __init__(self, field: str, value: Any, reason: str = None):
        message = f"Invalid task input: field '{field}' with value '{value}'"
        if reason:
            message += f" ({reason})"
        super().__init__(message, {"field": field, "value": str(value), "reason": reason})
        self.field = field
        self.value = value
        self.reason = reason


# 配置相关异常
class ConfigurationError(SchedulerError):
    """配置错误异常"""

    def __init__(self, config_key: str, reason: str):
        super().__init__(f"Configuration error for '{config_key}': {reason}", {"config_key": config_key, "reason": reason})
        self.config_key = config_key
        self.reason = reason


class SystemNotInitializedError(SchedulerError):
    """系统未初始化异常"""

    def __init__(self):
        super().__init__("System not initialized. Please call initialize-map first.")


class SystemAlreadyInitializedError(SchedulerError):
    """系统已初始化异常"""

    def __init__(self, campus: str):
        super().__init__(f"System already initialized for campus '{campus}'. Please reset first.", {"campus": campus})
        self.campus = campus


# 数据相关异常
class DataLoadError(SchedulerError):
    """数据加载异常"""

    def __init__(self, file_path: str, reason: str):
        super().__init__(f"Failed to load data from '{file_path}': {reason}", {"file_path": file_path, "reason": reason})
        self.file_path = file_path
        self.reason = reason


class YAMLValidationError(SchedulerError):
    """YAML 验证异常"""

    def __init__(self, file_path: str, errors: List[str]):
        error_list = "; ".join(errors)
        super().__init__(f"YAML validation failed for '{file_path}': {error_list}", {"file_path": file_path, "errors": errors})
        self.file_path = file_path
        self.errors = errors