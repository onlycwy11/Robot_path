import json
import math
from typing import List, Dict, Tuple, Any, Optional
import heapq

# ============================
# 1. 数据结构和配置
# ============================

class Position:
    """位置坐标"""
    def __init__(self, x: float, y: float, z: float):
        self.x = x
        self.y = y
        self.z = z
    
    def distance_to(self, other: 'Position') -> float:
        """计算两个位置之间的欧几里得距离"""
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2 + (self.z - other.z)**2)
    
    def __repr__(self):
        return f"({self.x}, {self.y}, {self.z})"

class Node:
    """地图节点"""
    def __init__(self, node_id: int, name: str, position: Position, node_type: str, 
                 building: str, floor_num: int = 1):  # 改为 floor_num 避免命名冲突
        self.node_id = node_id
        self.name = name
        self.position = position
        self.type = node_type
        self.building = building
        self.floor_num = floor_num  # 使用 floor_num 而不是 floor

class Edge:
    """边连接"""
    def __init__(self, from_node: int, to_node: int, distance: float, edge_type: str):
        self.from_node = from_node
        self.to_node = to_node
        self.distance = distance
        self.type = edge_type

class Graph:
    """地图图结构"""
    def __init__(self):
        self.nodes = {}
        self.edges = []
        self.adjacency_list = {}
    
    def add_node(self, node: Node):
        self.nodes[node.node_id] = node
        self.adjacency_list[node.node_id] = []
    
    def add_edge(self, edge: Edge):
        self.edges.append(edge)
        self.adjacency_list[edge.from_node].append((edge.to_node, edge.distance, edge.type))
        if edge.type != 'elevator':
            self.adjacency_list[edge.to_node].append((edge.from_node, edge.distance, edge.type))

class Elevator:
    """电梯 - 只能同时容纳一个机器人"""
    def __init__(self, elevator_id: int, name: str, building: str, position: Position, 
                 speed: float, door_time: float, initial_floor: int):
        self.elevator_id = elevator_id
        self.name = name
        self.building = building
        self.position = position
        self.speed = speed
        self.door_time = door_time
        self.current_floor = initial_floor
        self.state = 'idle'
        self.schedule = []
    
    def check_availability(self, desired_start_time: float, duration: float) -> Tuple[bool, float]:
        desired_end_time = desired_start_time + duration
        
        if not self.schedule:
            return True, desired_start_time
        
        for schedule in self.schedule:
            scheduled_start, scheduled_end, _, _, _ = schedule
            
            if not (desired_end_time <= scheduled_start or desired_start_time >= scheduled_end):
                desired_start_time = scheduled_end
                desired_end_time = desired_start_time + duration
        
        return True, desired_start_time
    
    def reserve(self, start_time: float, end_time: float, from_floor: int, to_floor: int, robot_id: int):
        self.schedule.append((start_time, end_time, from_floor, to_floor, robot_id))
        self.schedule.sort(key=lambda x: x[0])

class Robot:
    """机器人"""
    def __init__(self, robot_id: int, name: str, robot_type: str, 
                 speed_flat: float, speed_stairs: float, capacity: int, 
                 initial_position: Position, skill: str = "transport"):
        self.robot_id = robot_id
        self.name = name
        self.type = robot_type
        self.speed_flat = speed_flat
        self.speed_stairs = speed_stairs
        self.capacity = capacity
        self.position = initial_position
        self.skill = skill
        self.available_time = 0.0
        self.current_task = None

class Task:
    """任务"""
    def __init__(self, task_id: int, name: str, start_pos: Position, 
                 end_pos: Position, pickup_time: float, dropoff_time: float, 
                 required_space: float, skill: str = "transport"):
        self.task_id = task_id
        self.name = name
        self.start_pos = start_pos
        self.end_pos = end_pos
        self.pickup_time = pickup_time
        self.dropoff_time = dropoff_time
        self.required_space = required_space
        self.skill = skill

# ============================
# 2. 简化版建筑结构生成（避免复杂计算）
# ============================

def create_simple_graph():
    """创建一个图结构用于测试"""
    graph = Graph()
    
    # 添加一些测试节点
    nodes_data = [
        (1, "1号楼_1F_房间_A", Position(6000, 10000, 110), "room", "1号楼", 1),
        (2, "1号楼_2F_房间_B", Position(7500, 10000, 410), "room", "1号楼", 2),
        (3, "1号楼_1F_走廊", Position(8000, 12000, 110), "corridor", "1号楼", 1),
        (4, "1号楼_2F_走廊", Position(8000, 12000, 410), "corridor", "1号楼", 2),
        (5, "1号楼_1F_楼梯", Position(7000, 13000, 110), "stair", "1号楼", 1),
        (6, "1号楼_2F_楼梯", Position(7000, 13000, 410), "stair", "1号楼", 2),
        (7, "1号楼_1F_电梯", Position(9000, 12000, 110), "elevator", "1号楼", 1),
        (8, "1号楼_2F_电梯", Position(9000, 12000, 410), "elevator", "1号楼", 2),
    ]
    
    for node_id, name, position, node_type, building, floor_num in nodes_data:
        node = Node(node_id, name, position, node_type, building, floor_num)
        graph.add_node(node)
    
    # 添加边
    edges_data = [
        (1, 3, 500, "ordinary"),   # 房间A到走廊
        (2, 4, 500, "ordinary"),   # 房间B到走廊
        (3, 5, 300, "ordinary"),   # 走廊到楼梯
        (4, 6, 300, "ordinary"),   # 走廊到楼梯
        (3, 7, 400, "ordinary"),   # 走廊到电梯
        (4, 8, 400, "ordinary"),   # 走廊到电梯
        (5, 6, 300, "stair"),      # 楼梯连接
        (7, 8, 300, "elevator"),   # 电梯连接
    ]
    
    for from_node, to_node, distance, edge_type in edges_data:
        edge = Edge(from_node, to_node, distance, edge_type)
        graph.add_edge(edge)
    
    return graph

# ============================
# 3. 数据加载
# ============================

def load_data_from_files():
    """从提供的文件数据加载配置"""
    buildings = [
        {"buildingName": "1号楼", "buildingLayerSize": 3, "buildingNeighboringAverageDistance": 3.5},
        {"buildingName": "2号楼", "buildingLayerSize": 9, "buildingNeighboringAverageDistance": 3.5},
        {"buildingName": "3号楼", "buildingLayerSize": 6, "buildingNeighboringAverageDistance": 3.5}
    ]
    
    elevators = [
        {"elevatorName": "1号楼电梯一", "buildingName": "1号楼", "averageSpeed": 2.0, 
         "doorOperationTime": 10, "position": {"x": 38000, "y": 15000, "z": 110}, "initialLayer": 1},
        {"elevatorName": "1号楼电梯二", "buildingName": "1号楼", "averageSpeed": 2.0, 
         "doorOperationTime": 10, "position": {"x": 41000, "y": 15000, "z": 110}, "initialLayer": 1},
    ]
    
    robots = [
        {"robotName": "JK01", "type": "quadruped", "averageSpeedFlat": 1.5, 
         "averageSpeedStairs": 0.5, "capacity": 30, 
         "initialPosition": {"x": 6000, "y": 10000, "z": 110}},
        {"robotName": "GO2W", "type": "quadruped", "averageSpeedFlat": 1.5, 
         "averageSpeedStairs": 0.5, "capacity": 30, 
         "initialPosition": {"x": 6000, "y": 10000, "z": 110}}
    ]
    
    tasks = [
        {"taskName": "同层运输", "startPosition": {"x": 6000, "y": 10000, "z": 110}, 
         "endPosition": {"x": 7500, "y": 10000, "z": 110}, "pickupTime": 45, 
         "dropoffTime": 45, "requiredSpace": 70},
        {"taskName": "跨楼层运输", "startPosition": {"x": 6000, "y": 10000, "z": 110}, 
         "endPosition": {"x": 7500, "y": 10000, "z": 410}, "pickupTime": 45, 
         "dropoffTime": 45, "requiredSpace": 70},
    ]
    
    return buildings, elevators, robots, tasks

# ============================
# 4. 路径规划和时间计算
# ============================

def calculate_travel_time(robot: Robot, distance: float, edge_type: str) -> float:
    """根据边类型计算机器人旅行时间"""
    if edge_type in ['ordinary', 'corridor', 'road', 'bridge']:
        return distance / robot.speed_flat
    elif edge_type == 'stair':
        return distance / robot.speed_stairs
    elif edge_type == 'elevator':
        return distance / robot.speed_flat
    return distance / robot.speed_flat

def calculate_elevator_time(elevator: Elevator, from_floor: int, to_floor: int, 
                          robot_arrival_time: float, robot_id: int) -> Tuple[float, float, float]:
    """计算电梯旅行时间"""
    floor_height = 3.0
    floor_diff = abs(to_floor - from_floor)
    
    travel_distance = floor_diff * floor_height
    travel_time = travel_distance / elevator.speed
    door_time = elevator.door_time * 2
    elevator_total_time = travel_time + door_time
    
    available, actual_start_time = elevator.check_availability(robot_arrival_time, elevator_total_time)
    wait_time = actual_start_time - robot_arrival_time
    total_time = wait_time + elevator_total_time
    
    elevator.reserve(actual_start_time, actual_start_time + elevator_total_time, 
                    from_floor, to_floor, robot_id)
    
    return total_time, wait_time, elevator_total_time

def find_closest_node(graph: Graph, position: Position, max_distance: float = 10000) -> Optional[int]:
    """找到距离给定位置最近的节点"""
    closest_node = None
    min_distance = float('inf')
    
    for node_id, node in graph.nodes.items():
        distance = node.position.distance_to(position)
        if distance < min_distance and distance <= max_distance:
            min_distance = distance
            closest_node = node_id
    
    return closest_node

def shortest_path_time(robot: Robot, start_pos: Position, target_pos: Position, 
                      graph: Graph, elevators: Dict[int, Elevator], 
                      current_time: float = 0) -> Tuple[float, List[int], str]:
    """
    计算最短时间路径，比较所有选项
    """
    start_node = find_closest_node(graph, start_pos)
    target_node = find_closest_node(graph, target_pos)
    
    if start_node is None or target_node is None:
        return float('inf'), [], "无路径"
    
    # Dijkstra算法
    dist = {node_id: float('inf') for node_id in graph.nodes}
    prev = {node_id: None for node_id in graph.nodes}
    dist[start_node] = current_time
    pq = [(current_time, start_node)]
    
    while pq:
        current_time_at_node, current_node = heapq.heappop(pq)
        
        if current_time_at_node > dist[current_node]:
            continue
            
        if current_node == target_node:
            break
        
        for neighbor, distance, edge_type in graph.adjacency_list[current_node]:
            if edge_type == 'elevator':
                # 电梯特殊处理
                from_floor = graph.nodes[current_node].floor_num
                to_floor = graph.nodes[neighbor].floor_num
                
                # 使用第一个电梯（简化）
                elevator = list(elevators.values())[0]
                elevator_total_time, wait_time, _ = calculate_elevator_time(
                    elevator, from_floor, to_floor, current_time_at_node, robot.robot_id
                )
                arrival_time = current_time_at_node + elevator_total_time
            else:
                # 普通通道
                travel_time = calculate_travel_time(robot, distance, edge_type)
                arrival_time = current_time_at_node + travel_time
            
            if arrival_time < dist[neighbor]:
                dist[neighbor] = arrival_time
                prev[neighbor] = current_node
                heapq.heappush(pq, (arrival_time, neighbor))
    
    # 重建路径
    path = []
    current = target_node
    while current is not None:
        path.append(current)
        current = prev[current]
    path.reverse()
    
    total_time = dist[target_node] - current_time
    
    # 判断主要交通方式
    used_elevator = False
    for node_id in path:
        if graph.nodes[node_id].type == 'elevator':
            used_elevator = True
            break
    
    method = "电梯" if used_elevator else "楼梯"
    
    return total_time, path, method

# ============================
# 5. 调度模块
# ============================

def calculate_task_completion_time(robot: Robot, task: Task, graph: Graph, 
                                 elevators: Dict[int, Elevator]) -> Tuple[float, str]:
    """计算机器人完成任务的最终时间"""
    # 1. 机器人从当前位置移动到任务起点的时间
    time_to_start, path1, method1 = shortest_path_time(
        robot, robot.position, task.start_pos, graph, elevators, robot.available_time
    )
    
    # 2. 拾取时间
    pickup_time = task.pickup_time
    
    # 3. 机器人从任务起点移动到任务终点的时间
    time_to_target, path2, method2 = shortest_path_time(
        robot, task.start_pos, task.end_pos, graph, elevators, 
        robot.available_time + time_to_start + pickup_time
    )
    
    # 4. 放置时间
    dropoff_time = task.dropoff_time
    
    # 5. 总完成时间
    total_completion_time = (
        robot.available_time + 
        time_to_start + 
        pickup_time + 
        time_to_target + 
        dropoff_time
    )
    
    # 确定主要交通方式
    main_method = method2 if "电梯" in method2 else method1
    
    return total_completion_time, main_method

def assign_tasks(tasks: List[Task], robots: List[Robot], graph: Graph, 
                elevators: Dict[int, Elevator]) -> Dict[Task, Tuple[Robot, str]]:
    """将任务分配给能最早完成该任务的机器人"""
    assignment = {}
    
    for task in tasks:
        print(f"\n处理任务: {task.name}")
        
        candidate_robots = [r for r in robots if r.skill == task.skill]
        
        if not candidate_robots:
            print(f"警告: 没有具备技能 {task.skill} 的机器人可用于任务 {task.name}")
            continue
        
        best_robot = None
        earliest_completion_time = float('inf')
        best_method = ""
        
        for robot in candidate_robots:
            completion_time, method = calculate_task_completion_time(robot, task, graph, elevators)
            
            print(f"  机器人 {robot.name}: "
                  f"当前可用时间 {robot.available_time:.2f}s, "
                  f"预计完成时间 {completion_time:.2f}s, "
                  f"交通方式: {method}")
            
            if completion_time < earliest_completion_time:
                earliest_completion_time = completion_time
                best_robot = robot
                best_method = method
        
        assignment[task] = (best_robot, best_method)
        
        # 更新机器人的状态
        best_robot.available_time = earliest_completion_time
        best_robot.position = task.end_pos
        
        print(f"任务 {task.name} 分配给机器人 {best_robot.name} "
              f"(完成时间: {earliest_completion_time:.2f}s, 交通方式: {best_method})")
    
    return assignment

# ============================
# 6. 主程序
# ============================

def main():
    print("初始化多机器人任务调度系统...")
    
    # 加载数据
    buildings_data, elevators_data, robots_data, tasks_data = load_data_from_files()
    
    # 创建简化图结构
    graph = create_simple_graph()
    
    # 创建电梯
    elevators = {}
    for i, elevator_data in enumerate(elevators_data):
        pos_data = elevator_data["position"]
        position = Position(pos_data["x"], pos_data["y"], pos_data["z"])
        elevator = Elevator(
            elevator_id=i+1,
            name=elevator_data["elevatorName"],
            building=elevator_data["buildingName"],
            position=position,
            speed=elevator_data["averageSpeed"],
            door_time=elevator_data["doorOperationTime"],
            initial_floor=elevator_data["initialLayer"]
        )
        elevators[i+1] = elevator
    
    # 创建机器人
    robots = []
    for i, robot_data in enumerate(robots_data):
        pos_data = robot_data["initialPosition"]
        position = Position(pos_data["x"], pos_data["y"], pos_data["z"])
        robot = Robot(
            robot_id=i+1,
            name=robot_data["robotName"],
            robot_type=robot_data["type"],
            speed_flat=robot_data["averageSpeedFlat"],
            speed_stairs=robot_data["averageSpeedStairs"],
            capacity=robot_data["capacity"],
            initial_position=position
        )
        robots.append(robot)
    
    # 创建任务
    tasks = []
    for i, task_data in enumerate(tasks_data):
        start_pos_data = task_data["startPosition"]
        end_pos_data = task_data["endPosition"]
        
        start_pos = Position(start_pos_data["x"], start_pos_data["y"], start_pos_data["z"])
        end_pos = Position(end_pos_data["x"], end_pos_data["y"], end_pos_data["z"])
        
        task = Task(
            task_id=i+1,
            name=task_data["taskName"],
            start_pos=start_pos,
            end_pos=end_pos,
            pickup_time=task_data["pickupTime"],
            dropoff_time=task_data["dropoffTime"],
            required_space=task_data["requiredSpace"]
        )
        tasks.append(task)
    
    print(f"\n系统初始化完成:")
    print(f"- 地图节点: {len(graph.nodes)} 个")
    print(f"- 地图边: {len(graph.edges)} 条")
    print(f"- 电梯: {len(elevators)} 部") 
    print(f"- 机器人: {len(robots)} 台")
    print(f"- 任务: {len(tasks)} 个")
    
    # 执行任务分配
    assignment = assign_tasks(tasks, robots, graph, elevators)
    
    # 输出最终结果
    print("\n" + "="*60)
    print("最终任务分配结果:")
    print("="*60)
    for task, (robot, method) in assignment.items():
        print(f"任务: {task.name}")
        print(f"  分配给: {robot.name}")
        print(f"  完成时间: {robot.available_time:.2f} 秒")
        print(f"  交通方式: {method}")
        print(f"  起点: {task.start_pos}")
        print(f"  终点: {task.end_pos}")
        print()
    
    print("\n机器人最终状态:")
    for robot in robots:
        print(f"{robot.name}: 可用时间 = {robot.available_time:.2f}s, 位置 = {robot.position}")

if __name__ == "__main__":
    main()
    # import matplotlib
    # matplotlib.use('TkAgg')  # 或 'Qt5Agg', 'GTK3Agg' 等
    # import matplotlib.pyplot as plt
    #
    # local_coords = {
    #     'A': (60.00, 100.00), 'Ar': (60.00, 113.000),
    #     'B': (75.00, 100.00), 'Br': (75.00, 113.00),
    #     'C': (143.00, 100.00), 'Cr': (143.00, 113.00),
    #     'D': (220.00, 100.00), 'Dr': (220.00, 113.00),
    #     'E': (77.00, 113.00), 'Er': (77.00, 100.00),
    #     'F': (200.00, 113.00), 'Fr': (200.00, 100.00),
    #     'G': (293.00, 113.00), 'Gr': (293.00, 100.00),
    #     'Left_1': (3.00, 113.00), 'Left_2': (3.00, 100.00),
    #     'Right_1': (433.00, 113.00), 'Right_2': (433.00, 100.00),
    #     'E1': (380.00, 150.00), 'E2': (405.00, 150.00),
    #     'Stair1_1': (93.00, 150.00), 'Stair1_2': (122.00, 150.00),
    #     'Stair2_1': (380.00, 199.00), 'Stair2_2': (380.00, 167.00),
    # }
    #
    # plt.figure(figsize=(10, 8))
    #
    # # 绘制所有点
    # for name, (x, y) in local_coords.items():
    #     plt.scatter(x, y, label=name)
    #     plt.text(x + 2, y, name, fontsize=9)
    #
    # # 连接相关点（例如A-Ar, B-Br等）
    # for base in ['A', 'B', 'C', 'D', 'E', 'F', 'G']:
    #     x, y = local_coords[base]
    #     xr, yr = local_coords[base + 'r']
    #     plt.plot([x, xr], [y, yr], 'k--', alpha=0.3)
    #
    # # 设置坐标轴范围
    # plt.xlim(0, 450)
    # plt.ylim(90, 210)
    # plt.gca().set_aspect('equal')
    # plt.grid(True)
    # plt.title("Local Coordinates Layout")
    # plt.xlabel("X")
    # plt.ylabel("Y")
    # plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    # plt.tight_layout()
    # plt.show()