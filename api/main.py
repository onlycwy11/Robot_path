"""
机器人集群调度系统 API

FastAPI 主入口，仅负责路由注册和全局配置。
具体实现下沉到 routers 模块中。

结构：
- main.py: 路由注册、全局配置
- routers/: 各模块路由实现
- schemas/: Pydantic 模型定义
- core/: API 辅助函数
- state.py: 全局状态管理（集成 MQTT）

版本：v2.1.0
"""

from fastapi import FastAPI

from api.routers import system_router, map_router, scheduler_router, robot_router
from api.schemas.system import RootResponse

# 创建 FastAPI 应用
app = FastAPI(
    title="Robot Cluster Scheduler API",
    description="""
多机器人集群调度系统 API

## 功能模块

- **System**: 系统状态、健康检查、重置
- **Map**: 地图初始化、节点查询、电梯查询
- **Scheduler**: 任务调度、取消、重调度
- **Robots**: 机器人状态查询（MQTT 实时数据）

## 数据来源

机器人状态优先从 MQTT 获取实时数据。
若 MQTT 无数据，字段值为空（null）。

## 快速开始

1. `POST /map/initialize` - 初始化地图和机器人
2. `POST /scheduler/schedule` - 调度任务
3. `GET /robots/status` - 查询机器人实时状态
""",
    version="2.1.0",
    docs_url="/docs",
    redoc_url="/redoc"
)

# 注册路由
app.include_router(system_router)
app.include_router(map_router)
app.include_router(scheduler_router)
app.include_router(robot_router)



@app.on_event("startup")
async def startup_event():
    """应用启动时初始化 MQTT 连接"""
    from api.state import global_state
    from src.utils.config import DEFAULT_CONFIG
    global_state.start_mqtt(config=DEFAULT_CONFIG)


@app.on_event("shutdown")
async def shutdown_event():
    """应用关闭时清理资源"""
    from api.state import global_state
    global_state.stop_mqtt()