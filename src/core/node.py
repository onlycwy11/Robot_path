import math

# 局部相对坐标（单位：cm 或自定义单位）
local_coords = {
    'A': (60.00, 100.00), 'Ar': (60.00, 113.000),
    'B': (75.00, 100.00), 'Br': (75.00, 113.00),
    'C': (143.00, 100.00), 'Cr': (143.00, 113.00),
    'D': (220.00, 100.00), 'Dr': (220.00, 113.00),
    'E': (77.00, 113.00), 'Er': (77.00, 100.00),
    'F': (200.00, 113.00), 'Fr': (200.00, 100.00),
    'G': (293.00, 113.00), 'Gr': (293.00, 100.00),
    'Left_1': (3.00, 113.00), 'Left_2': (3.00, 100.00),
    'Right_1': (433.00, 113.00), 'Right_2': (433.00, 100.00),
    'E1': (380.00, 150.00), 'E2': (405.00, 150.00),
    'Stair1_1': (93.00, 150.00), 'Stair1_2': (122.00, 150.00),
    'Stair2_1': (380.00, 199.00), 'Stair2_2': (380.00, 167.00),
}


def get_coordinates_from_node(node: str):
    """将节点名解析为全局坐标 (cm，整数)"""
    try:
        parts = node.split('_')
        floor = int(parts[0])
        building = int(parts[1])
        room = parts[2]
        if len(parts) == 4:
            room = parts[2] + '_' + parts[3]
    except Exception:
        print(f"节点 {node} 格式错误！")
        return None

    if room not in local_coords:
        print(f"房间 {room} 未定义！")
        return None

    local_x, local_y = local_coords[room]
    x = local_x + (building - 1) * 633.00
    y = local_y
    z = 1.10 + (floor - 1) * 3.50
    return (int(round(x * 100)), int(round(y * 100)), int(round(z * 100)))  # 转为 cm，整数


def show_path_with_coords(path_list):
    # print("Path with Coordinates:")
    new_path = []
    for node in path_list:
        coord = get_coordinates_from_node(node)
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


def get_path_points(path_list):
    """返回每 1 秒的路径点 (x, y, z, t)，坐标均为整数"""
    coords = [get_coordinates_from_node(n) for n in path_list]
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


def get_xyz_from_path_and_time(path_list, t: float):
    """
    输入路径 path_list 和时间 t（秒），返回当前 (x, y, z) 坐标（整数）。
    若超过总时间，返回终点坐标。
    """
    # 先生成整条路径的 (x, y, z, t)
    path_points = get_path_points(path_list)
    # 使用已有函数查询指定时间的坐标
    pos = get_position_at_time(t, path_points)
    return pos


def get_xyz_from_path_and_time_with_elevator_wait(
        path_list, t: float, wait_time_1: float = 0.0, wait_time_2: float = 0.0):
    """
    路径 path_list 和时间 t（秒） -> 返回当前 (x, y, z) 坐标。
    如果路径中有 E1/E2 节点：
    - 在第一组 E1/E2 节点停留 wait_time_1 秒，
    - 在第二组 E1/E2 节点停留 wait_time_2 秒。
    - 如果只有一组 E1/E2 节点，则只在它们之间停留 wait_time_2 秒。
    """
    path_points = []
    coords = [get_coordinates_from_node(n) for n in path_list]
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
    # nodes = ['1_1_Left_1', '1_1_Left_2', '1_1_A', '1_1_B', '1_1_Er',
    #          '1_1_C', '1_1_Fr', '1_1_D', '1_1_Gr', '1_1_Right_2',
    #          '1_2_Left_2', '1_2_A', '1_2_B', '1_2_Er', '1_2_C',
    #          '1_2_Fr', '1_2_D', '1_2_Gr', '1_2_Right_2',
    #          '1_3_Left_2', '1_3_A', '1_3_B', '1_3_Er', '1_3_C',
    #          '1_3_Fr', '1_3_D', '1_3_Gr', '1_3_E1', '3_3_E1', '3_3_G']
    # nodes = ['1_3_E1', '3_3_E1', '3_3_G']
    # nodes = ['4_3_G', '4_3_Gr', '4_3_E1', '1_3_E1', '1_3_G', '1_3_Dr',
    #          '1_3_F', '1_3_Cr', '1_3_E', '1_3_Br', '1_3_Ar', '1_3_Left_1',
    #          '1_2_Right_1', '1_2_E2', '6_2_E2', '6_2_G']
    # nodes = ['1_2_E2', '6_2_E2']
    nodes = ['1_2_Stair1_2', '2_2_Stair1_2', '3_2_Stair1_2', '4_2_Stair1_2', '5_2_Stair1_2', '6_2_Stair1_2', '7_2_Stair1_2', '8_2_Stair1_2', '8_2_Stair1_1', '8_2_E']
    path_pts = get_path_points(nodes)
    print(f"路径{show_path_with_coords(nodes)}")
    # print(f"共生成 {len(path_pts)} 个点，总时长约 {path_pts[-1][3]} 秒。")
    # print (path_pts)
    # for test_t in [0, 9.5, 10, 50, 100,300, 500, 1000, 1100,1110,1120,1130, 1140, 1150, 1160, 1170, 1175, 1180, 1190,1200, 1300, 1500, 2000]:
    #     xyz = get_xyz_from_path_and_time(nodes, test_t)
    #     print(f"t={test_t:6.2f}s -> 位置: (x={xyz[0]}, y={xyz[1]}, z={xyz[2]})")

    # for test_t in [0, 9.5, 10, 30, 31, 32, 50, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88,
    #                89, 90, 100, 300, 500, 510, 520, 530, 531, 535, 538, 539, 540, 546, 547, 548, 550,
    #                560, 570, 580, 590, 600, 660, 700]:
    for test_t in [0, 3, 5, 6, 7, 8, 9.5, 10, 11, 12, 13, 14, 15, 18, 30]:
        xyz = get_xyz_from_path_and_time_with_elevator_wait(nodes, test_t, 0, 5)
        # xyz = get_xyz_from_path_and_time_with_elevator_wait(nodes, test_t, 5, 30)
        print(f"t={test_t:6.2f}s -> 位置: (x={xyz[0]}, y={xyz[1]}, z={xyz[2]})")
