"""
API 路由模块

提供模块化的路由定义。
"""

from api.routers.system import router as system_router
from api.routers.map import router as map_router
from api.routers.scheduler import router as scheduler_router
from api.routers.robot import router as robot_router

__all__ = [
    "system_router",
    "map_router",
    "scheduler_router",
    "robot_router",
]