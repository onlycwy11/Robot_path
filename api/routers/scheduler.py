"""
调度路由模块

提供任务调度、取消、查询等接口。
"""

from fastapi import APIRouter, HTTPException, Query
from typing import List, Optional
import time

from src.models.task import Task
from src.utils.path_utils import merge_paths

from api.schemas.task import (
    BatchTaskInput,
    AssignmentOutput,
    TaskCancelResponse,
)
from api.state import global_state

router = APIRouter(prefix="/scheduler", tags=["Scheduler"])


@router.post("/schedule", response_model=List[AssignmentOutput],description="批量调度任务")
async def schedule_tasks(batch_input: BatchTaskInput):
    """
    根据任务优先级、机器人技能和位置，自动分配最优机器人执行任务。
    支持电梯预约冲突检测和解决。
    """
    if not global_state.initialized:
        raise HTTPException(
            status_code=400,
            detail={
                "errorCode": "NOT_INITIALIZED",
                "errorMessage": "系统未初始化，请先调用 /map/initialize",
                "hint": "POST /map/initialize 初始化地图后再调度任务"
            }
        )

    scheduler = global_state.batch_scheduler

    try:
        # 转换输入为 Task 对象
        batch_tasks = [
            Task(
                id=task_input.task_id,
                skill=task_input.skill,
                start=task_input.start_position,
                target=task_input.target_position,
                priority=task_input.priority
            )
            for task_input in batch_input.tasks
        ]

        # 执行调度
        assignments = scheduler.schedule_batch(batch_tasks)

        # 格式化输出
        output = []
        for assignment in assignments:
            path1 = assignment["pick_path_info"]["path"]
            path2 = assignment["deliver_path_info"]["path"]

            full_path = merge_paths(path1, path2)
            estimated_time = assignment["end_time"] - assignment["start_time"]

            output.append(AssignmentOutput(
                task_id=assignment["task"].id,
                robot_id=assignment["robot_id"],
                path=full_path,
                estimated_time=estimated_time
            ))

        return output

    except Exception as e:
        raise HTTPException(
            status_code=400,
            detail={"errorCode": "SCHEDULE_FAILED", "errorMessage": str(e)}
        )


@router.get("/tasks",summary="获取所有任务")
async def get_all_tasks(
    robot_id: Optional[int] = Query(None, description="筛选指定机器人的任务"),
    status: Optional[str] = Query(None, description="筛选任务状态：running/completed/pending")
):
    """
    获取所有任务或指定机器人的任务
    """
    if not global_state.initialized:
        raise HTTPException(
            status_code=400,
            detail={"errorCode": "NOT_INITIALIZED", "errorMessage": "系统未初始化"}
        )

    scheduler = global_state.batch_scheduler

    # 确定查询范围
    if robot_id is not None:
        robots_to_query = [scheduler._find_robot_by_id(robot_id)]
        if not robots_to_query[0]:
            raise HTTPException(
                status_code=404,
                detail={"errorCode": "ROBOT_NOT_FOUND", "errorMessage": f"机器人 {robot_id} 不存在"}
            )
    else:
        robots_to_query = scheduler.robots

    tasks = []
    for robot in robots_to_query:
        if robot:
            for task_id, task_info in robot.task_list.items():
                task_status = "running" if time.time() - scheduler.start_time < task_info.finish_time else "completed"
                if status and task_status != status:
                    continue
                tasks.append({
                    "taskId": task_id,
                    "robotId": robot.id,
                    "status": task_status,
                    "startTime": task_info.start_time,
                    "finishTime": task_info.finish_time,
                    "pathLength": len(task_info.path)
                })

    return {
        "schemaVersion": "v1",
        "messageType": "tasks_list_response",
        "tasks": tasks,
        "total": len(tasks)
    }


@router.delete("/tasks/{task_id}", response_model=TaskCancelResponse,summary="取消任务")
async def cancel_task(task_id: int):
    """
    取消正在执行或待执行的任务，释放电梯预约资源。
    """
    if not global_state.initialized:
        raise HTTPException(
            status_code=400,
            detail={"errorCode": "NOT_INITIALIZED", "errorMessage": "系统未初始化"}
        )

    scheduler = global_state.batch_scheduler

    try:
        success = scheduler.cancel_task(task_id)

        if success:
            return TaskCancelResponse(
                task_id=task_id,
                cancelled=True,
                message=f"Task {task_id} cancelled successfully"
            )
        else:
            return TaskCancelResponse(
                task_id=task_id,
                cancelled=False,
                message=f"Task {task_id} not found or already completed"
            )

    except Exception as e:
        raise HTTPException(
            status_code=400,
            detail={"errorCode": "CANCEL_FAILED", "errorMessage": str(e)}
        )

