from src.core.graph import Graph, inital_graph

def find_paths_with_different_methods(graph, start, end):
    """
    比较只使用楼梯和只使用电梯时的最短路径
    """
    print(f"\n{'=' * 60}")
    print(f"路径规划: {start} → {end}")
    print(f"{'=' * 60}")

    # 方法1: 只使用楼梯 (移除电梯边)
    stair_graph = create_stair_only_graph(graph)
    stair_path, stair_cost = stair_graph.dijkstra(start, end)

    # 方法2: 只使用电梯 (移除楼梯边)
    elevator_graph = create_elevator_only_graph(graph)
    elevator_path, elevator_cost = elevator_graph.dijkstra(start, end)

    # 显示结果比较
    print(f"\n📊 路径比较结果:")
    print(f"{'=' * 40}")

    if stair_path:
        print(f"🚪 只使用楼梯:")
        print(f"   路径: {' → '.join(stair_path)}")
        print(f"   时间: {stair_cost:.2f} 秒 ({stair_cost / 60:.2f} 分钟)")
    else:
        print(f"🚪 只使用楼梯: 无可用路径")

    if elevator_path:
        print(f"🛗 只使用电梯:")
        print(f"   路径: {' → '.join(elevator_path)}")
        print(f"   时间: {elevator_cost:.2f} 秒 ({elevator_cost / 60:.2f} 分钟)")
    else:
        print(f"🛗 只使用电梯: 无可用路径")

    # 推荐最佳方案
    if stair_path and elevator_path:
        if stair_cost < elevator_cost:
            print(f"\n💡 推荐方案: 使用楼梯 (快 {elevator_cost - stair_cost:.2f} 秒)")
        else:
            print(f"\n💡 推荐方案: 使用电梯 (快 {stair_cost - elevator_cost:.2f} 秒)")
    elif stair_path:
        print(f"\n💡 推荐方案: 使用楼梯 (唯一可用)")
    elif elevator_path:
        print(f"\n💡 推荐方案: 使用电梯 (唯一可用)")
    else:
        print(f"\n❌ 无可用路径")

    return stair_path, stair_cost, elevator_path, elevator_cost


def create_stair_only_graph(original_graph):
    """创建只包含楼梯连接的图"""
    stair_graph = Graph()

    for node, neighbors in original_graph.edges.items():
        for neighbor, weight in neighbors:
            # 只保留楼梯连接 (包含Stair关键词) 和平地连接
            if ("Stair" in node and "Stair" in neighbor) or ("Stair" not in node and "Stair" not in neighbor):
                # 排除电梯连接
                if not is_elevator_connection(node, neighbor):
                    # 保持原有的双向性
                    is_bidirectional = is_bidirectional_edge(original_graph, node, neighbor)
                    stair_graph.add_edge(node, neighbor, weight, bidirectional=is_bidirectional)

    return stair_graph


def create_elevator_only_graph(original_graph):
    """创建只包含电梯连接的图"""
    elevator_graph = Graph()

    for node, neighbors in original_graph.edges.items():
        for neighbor, weight in neighbors:
            # 只保留电梯连接
            if is_elevator_connection(node, neighbor):
                # 电梯连接通常是双向的
                elevator_graph.add_edge(node, neighbor, weight, bidirectional=True)
            # 同时保留同一楼层内的平地连接
            elif is_same_floor_walk(node, neighbor):
                is_bidirectional = is_bidirectional_edge(original_graph, node, neighbor)
                elevator_graph.add_edge(node, neighbor, weight, bidirectional=is_bidirectional)

    return elevator_graph


def is_elevator_connection(node1, node2):
    """判断是否为电梯连接"""
    elevator_indicators = ["_E1", "_E2"]
    return any(indicator in node1 and indicator in node2 for indicator in elevator_indicators)


def is_same_floor_walk(node1, node2):
    """判断是否为同一楼层内的步行连接"""
    # 提取楼层信息 (格式: 楼层_楼号_位置)
    try:
        floor1 = node1.split('_')[0]
        floor2 = node2.split('_')[0]
        building1 = node1.split('_')[1]
        building2 = node2.split('_')[1]

        # 同一楼层同一建筑的连接
        return floor1 == floor2 and building1 == building2 and not is_elevator_connection(node1, node2)
    except:
        return False


def is_bidirectional_edge(graph, node1, node2):
    """判断原图中两个节点之间是否有双向连接"""
    forward = any(n == node2 for n, _ in graph.edges.get(node1, []))
    backward = any(n == node1 for n, _ in graph.edges.get(node2, []))
    return forward and backward


# ============================================================
# 测试用例
# ============================================================

if __name__ == "__main__":
    graph = inital_graph(speed_land=1.5, speed_stair=0.5)

    # 测试用例1: 同一楼层内的路径
    print("测试用例1: 同一楼层内移动")
    find_paths_with_different_methods(graph, "1_1_Left_2", "1_1_Ewr")

    # 测试用例2: 不同楼层同建筑
    print("\n\n测试用例2: 不同楼层同建筑")
    find_paths_with_different_methods(graph, "1_1_Left_2", "2_1_Ewr")

    # 测试用例3: 不同建筑不同楼层
    print("\n\n测试用例3: 跨建筑跨楼层")
    find_paths_with_different_methods(graph, "1_1_Left_2", "3_2_Ewr")

    # 测试用例4: 高层建筑
    print("\n\n测试用例4: 高层移动")
    find_paths_with_different_methods(graph, "1_2_Left_2", "9_2_Ewr")

    # 测试用例5: 电梯专用测试
    print("\n\n测试用例5: 电梯专用场景")
    find_paths_with_different_methods(graph, "1_3_E1", "6_3_E1")