"""
日志系统模块

提供统一的日志配置和管理，替换项目中的 print 语句。
支持不同模块的独立日志级别配置。
"""

import logging
import sys
from typing import Optional


# 日志格式
LOG_FORMAT = "%(asctime)s | %(levelname)-8s | %(name)s | %(message)s"
DATE_FORMAT = "%Y-%m-%d %H:%M:%S"


# 模块日志级别配置
MODULE_LOG_LEVELS = {
    "src.core.graph": logging.INFO,
    "src.core.node": logging.INFO,
    "src.schedulers": logging.INFO,
    "src.models": logging.INFO,
    "api": logging.INFO,
    "src.utils.mqtt_handler": logging.INFO,
}


def get_logger(name: str, level: Optional[int] = None) -> logging.Logger:
    """
    获取配置好的 Logger 实例

    Args:
        name: 模块名称（通常使用 __name__）
        level: 日志级别（可选，默认根据模块配置）

    Returns:
        配置好的 Logger 实例

    Example:
        logger = get_logger(__name__)
        logger.info("System initialized")
        logger.warning("Low battery: 20%")
        logger.error("Connection failed")
    """
    logger = logging.getLogger(name)

    # 设置级别
    if level is not None:
        logger.setLevel(level)
    elif name in MODULE_LOG_LEVELS:
        logger.setLevel(MODULE_LOG_LEVELS[name])
    else:
        logger.setLevel(logging.INFO)

    # 避免重复添加 handler
    if not logger.handlers:
        handler = logging.StreamHandler(sys.stdout)
        handler.setFormatter(logging.Formatter(LOG_FORMAT, DATE_FORMAT))
        logger.addHandler(handler)

    # 避免日志向上传播到 root logger
    logger.propagate = False

    return logger


def set_global_log_level(level: int):
    """
    设置全局日志级别

    Args:
        level: logging.DEBUG, INFO, WARNING, ERROR, CRITICAL
    """
    logging.getLogger().setLevel(level)
    for name in MODULE_LOG_LEVELS:
        logging.getLogger(name).setLevel(level)


def enable_debug_mode():
    """启用调试模式（所有模块 DEBUG 级别）"""
    set_global_log_level(logging.DEBUG)


def enable_quiet_mode():
    """启用静默模式（仅 ERROR 及以上）"""
    set_global_log_level(logging.ERROR)


# 预创建常用 logger
scheduler_logger = get_logger("src.schedulers")
graph_logger = get_logger("src.core.graph")
robot_logger = get_logger("src.models.robot")
mqtt_logger = get_logger("src.utils.mqtt_handler")
api_logger = get_logger("api")