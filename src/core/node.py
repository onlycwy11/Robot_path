import math
import os.path
from functools import lru_cache

import yaml

from src.utils.logger import graph_logger

BASE_PATH = os.path.join(os.path.dirname(__file__), "data")


@lru_cache(maxsize=8)
def _load_node_coord_cache(campus_name: str) -> dict:
    """
    读取 merged_nodes.yaml，建立 node_name -> (x_cm, y_cm, z_cm) 的索引

    使用 LRU 缓存支持多校园切换，最多缓存 8 个校园的数据。
    """
    yaml_path = os.path.join(BASE_PATH, campus_name, "merged_nodes.yaml")

    try:
        with open(yaml_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
    except FileNotFoundError:
        graph_logger.warning(f"merged_nodes.yaml 未找到: {yaml_path}")
        return {}
    except Exception as e:
        graph_logger.warning(f"读取 merged_nodes.yaml 失败：{e}")
        return {}

    node_list = data.get("node_list", None)
    if node_list is None:
        node_list = data.get("ode_list", None)

    if not isinstance(node_list, list):
        raise ValueError(f"merged_nodes.yaml 格式不对：缺少 node_list(或 ode_list) list，实际为 {type(node_list)}")

    cache = {}
    for n in node_list:
        name = n.get("node_name")
        if not name:
            continue

        c = n.get("node_coordinate", {}) or {}
        unit = str(c.get("unit", "m")).lower().strip()

        SCALE = 0.05
        x = float(c.get("x", 0.0)) * SCALE
        y = float(c.get("y", 0.0)) * SCALE
        z = float(c.get("z", 0.0)) * SCALE

        if unit in ("m", "meter", "meters"):
            x_cm = int(round(x * 100))
            y_cm = int(round(y * 100))
            z_cm = int(round(z * 100))
        elif unit in ("cm", "centimeter", "centimeters"):
            x_cm = int(round(x))
            y_cm = int(round(y))
            z_cm = int(round(z))
        else:
            x_cm = int(round(x * 100))
            y_cm = int(round(y * 100))
            z_cm = int(round(z * 100))

        cache[str(name)] = (x_cm, y_cm, z_cm)

    graph_logger.debug(f"加载校园 {campus_name} 节点数据: {len(cache)} 个节点")
    return cache


def clear_coordinate_cache():
    """清理坐标缓存（切换校园时调用）"""
    _load_node_coord_cache.cache_clear()
    graph_logger.info("坐标缓存已清理")


def get_cache_info() -> dict:
    """获取缓存状态信息"""
    info = _load_node_coord_cache.cache_info()
    return {
        "hits": info.hits,
        "misses": info.misses,
        "maxsize": info.maxsize,
        "currsize": info.currsize
    }


def get_coordinates_from_node(node: str, campus_name: str = "zheshang"):
    """
    从 merged_nodes.yaml 查表得到全局坐标 (cm, int)：
      return (x_cm, y_cm, z_cm)

    Args:
        node: 节点名称
        campus_name: 校园名称（支持多校园）

    Returns:
        (x_cm, y_cm, z_cm) 坐标三元组，或 None
    """
    cache = _load_node_coord_cache(campus_name)

    if node not in cache:
        graph_logger.warning(f"节点 {node} 未在 {campus_name} 的 merged_nodes.yaml 中定义！")
        return None

    return cache[node]

def show_path_with_coords(path_list, campus_name: str):
    # print("Path with Coordinates:")
    new_path = []
    for node in path_list:
        coord = get_coordinates_from_node(node, campus_name)
        new_path.append(coord)
        # if coord:
        #     print(f"{node:<12} -> 坐标 (x={coord[0]}, y={coord[1]}, z={coord[2]})")
    # print(new_path)
    return new_path


################################################ for show path
def get_speed(node1, node2):
    """根据节点类型确定速度（cm/s）"""
    if "E1" in node1 and "E1" in node2:
        return 200.0
    elif "E2" in node1 and "E2" in node2:
        return 200.0
    elif ("Stair1" in node1 and "Stair1" in node2) or ("Stair2" in node1 and "Stair2" in node2):
        return 50.0
    else:
        return 150.0


def get_path_points(path_list, campus_name: str):
    """返回每 1 秒的路径点 (x, y, z, t)，坐标均为整数"""
    coords = [get_coordinates_from_node(n, campus_name) for n in path_list]
    time_points = []
    total_time = 0.0

    for i in range(len(coords) - 1):
        x1, y1, z1 = coords[i]
        x2, y2, z2 = coords[i + 1]
        node1, node2 = path_list[i], path_list[i + 1]

        # 计算距离与速度
        dist = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)
        speed = get_speed(node1, node2)
        duration = dist / speed  # 该段时间长度（秒）

        steps = int(duration)
        for s in range(steps):
            ratio = s / duration if duration > 0 else 0
            xt = int(round(x1 + (x2 - x1) * ratio))
            yt = int(round(y1 + (y2 - y1) * ratio))
            zt = int(round(z1 + (z2 - z1) * ratio))
            time_points.append((xt, yt, zt, round(total_time + s, 2)))

        total_time += duration

    # 最后一个终点补上
    x2, y2, z2 = coords[-1]
    time_points.append((int(x2), int(y2), int(z2), round(total_time, 2)))
    return time_points


def get_position_at_time(t: float, path_points):
    """
    输入任意时间t（秒），返回当时的(x, y, z)坐标。
    若时间超出范围，返回首点或末点。
    """
    if t <= path_points[0][3]:
        return path_points[0][:3]
    if t >= path_points[-1][3]:
        return path_points[-1][:3]

    # 找到t所在的时间区间
    for i in range(len(path_points) - 1):
        x1, y1, z1, t1 = path_points[i]
        x2, y2, z2, t2 = path_points[i + 1]
        if t1 <= t <= t2:
            ratio = (t - t1) / (t2 - t1) if t2 > t1 else 0
            x = int(round(x1 + (x2 - x1) * ratio))
            y = int(round(y1 + (y2 - y1) * ratio))
            z = int(round(z1 + (z2 - z1) * ratio))
            return (x, y, z)
    return path_points[-1][:3]


def get_xyz_from_path_and_time(path_list, t: float, campus_name: str):
    """
    输入路径 path_list 和时间 t（秒），返回当前 (x, y, z) 坐标（整数）。
    若超过总时间，返回终点坐标。
    """
    # 先生成整条路径的 (x, y, z, t)
    path_points = get_path_points(path_list, campus_name)
    # 使用已有函数查询指定时间的坐标
    pos = get_position_at_time(t, path_points)
    return pos


def get_xyz_from_path_and_time_with_elevator_wait(
        path_list, t: float, wait_time_1: float = 0.0, wait_time_2: float = 0.0, campus_name: str = "zheshang"
):
    """
    路径 path_list 和时间 t（秒） -> 返回当前 (x, y, z) 坐标。
    如果路径中有 E1/E2 节点：
    - 在第一组 E1/E2 节点停留 wait_time_1 秒，
    - 在第二组 E1/E2 节点停留 wait_time_2 秒。
    - 如果只有一组 E1/E2 节点，则只在它们之间停留 wait_time_2 秒。
    """
    path_points = []
    coords = [get_coordinates_from_node(n, campus_name) for n in path_list]
    total_time = 0.0
    elevator_groups = []
    current_group = []

    # 首先识别电梯节点组
    for node in path_list:
        if "E1" in node or "E2" in node:
            current_group.append(node)
        else:
            if current_group:
                elevator_groups.append(current_group)
                current_group = []
    if current_group:
        elevator_groups.append(current_group)

    # 确定等待时间应用的位置
    wait_times = []
    if len(elevator_groups) >= 2:
        # 有两组或以上电梯节点
        wait_times = [wait_time_1, wait_time_2]
    elif len(elevator_groups) == 1 and len(elevator_groups[0]) >= 2:
        # 只有一组电梯节点
        wait_times = [wait_time_2]

    wait_inserted = 0  # 记录已经插入了多少个等待时间

    for i in range(len(coords) - 1):
        x1, y1, z1 = coords[i]
        x2, y2, z2 = coords[i + 1]
        node1, node2 = path_list[i], path_list[i + 1]

        # 检查是否需要在当前段之后插入等待时间
        if wait_inserted < len(wait_times):
            # 查找当前段是否属于电梯组之间的边界
            current_nodes_in_group = []
            # 向后查找属于同一电梯组的节点
            j = i
            while j + 1 < len(path_list) and (
                    ("E1" in path_list[j] and "E1" in path_list[j + 1]) or (
                    "E2" in path_list[j] and "E2" in path_list[j + 1])
            ):
                current_nodes_in_group.append(path_list[j])
                current_nodes_in_group.append(path_list[j + 1])
                j += 1

            # 如果这是电梯组的最后一个节点（即下一节点不是电梯节点）
            if current_nodes_in_group and j < len(path_list):
                if j + 1 < len(path_list) and not ("E1" in path_list[j + 1] or "E2" in path_list[j + 1]):
                    # 添加当前起点作为等待点
                    path_points.append((x1, y1, z1, round(total_time, 2)))
                    total_time += wait_times[wait_inserted]
                    wait_inserted += 1
                elif j + 1 == len(path_list):
                    # 添加当前起点作为等待点
                    path_points.append((x1, y1, z1, round(total_time, 2)))
                    total_time += wait_times[wait_inserted]
                    wait_inserted += 1

        # 计算这一段距离与速度
        dist = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)
        speed = get_speed(node1, node2)
        duration = dist / speed
        steps = int(duration)
        for s in range(steps):
            ratio = s / duration if duration > 0 else 0
            xt = int(round(x1 + (x2 - x1) * ratio))
            yt = int(round(y1 + (y2 - y1) * ratio))
            zt = int(round(z1 + (z2 - z1) * ratio))
            path_points.append((xt, yt, zt, round(total_time + s, 2)))

        total_time += duration

        # 最后一个节点补上
    x_end, y_end, z_end = coords[-1]
    path_points.append((int(x_end), int(y_end), int(z_end), round(total_time, 2)))

    # 查找时间 t 对应位置
    if t <= path_points[0][3]:
        return path_points[0][:3]
    if t >= path_points[-1][3]:
        return path_points[-1][:3]

    for i in range(len(path_points) - 1):
        x1, y1, z1, t1 = path_points[i]
        x2, y2, z2, t2 = path_points[i + 1]
        if t1 <= t <= t2:
            ratio = (t - t1) / (t2 - t1) if t2 > t1 else 0
            x = int(round(x1 + (x2 - x1) * ratio))
            y = int(round(y1 + (y2 - y1) * ratio))
            z = int(round(z1 + (z2 - z1) * ratio))
            return (x, y, z)

    return path_points[-1][:3]


if __name__ == "__main__":
    # 测试示例
    nodes = ['1_2_Stair1_2', '2_2_Stair1_2', '3_2_Stair1_2', '4_2_Stair1_2', '5_2_Stair1_2',
             '6_2_Stair1_2', '7_2_Stair1_2', '8_2_Stair1_2', '8_2_Stair1_1', '8_2_E']
    path_pts = get_path_points(nodes, 'sandun')
    graph_logger.info(f"路径坐标: {show_path_with_coords(nodes, 'sandun')}")
    for test_t in [0, 3, 5, 10, 15, 30]:
        xyz = get_xyz_from_path_and_time_with_elevator_wait(nodes, test_t, 0, 5)
        graph_logger.info(f"t={test_t:6.2f}s -> 位置: (x={xyz[0]}, y={xyz[1]}, z={xyz[2]})")
