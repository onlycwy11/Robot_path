def add_a_building_layer(graph, current_building, up_building, right_building, stair_length_1, stair_length_2,v_l,v_s):
    graph.add_edge(current_building+"Left_2", current_building+"A", (60-3)/v_l, bidirectional=False)
    graph.add_edge(current_building+"Left_2", current_building+"Left_1", (113-92)/v_l)
    graph.add_edge(current_building+"Ar", current_building+"Left_1", (60-3)/v_l, bidirectional=False)

    graph.add_edge(current_building+"A", current_building+"Ar", (113 - 92)/v_l)
    graph.add_edge(current_building+"B", current_building+"Br", (113 - 92) / v_l)
    graph.add_edge(current_building+"C", current_building+"Cr", (113 - 92) / v_l)
    graph.add_edge(current_building+"D", current_building+"Dr", (113 - 92) / v_l)
    graph.add_edge(current_building+"E", current_building+"Er", (113 - 92) / v_l)
    graph.add_edge(current_building+"F", current_building+"Fr", (113 - 92) / v_l)
    graph.add_edge(current_building+"G", current_building+"Gr", (113 - 92) / v_l)
    graph.add_edge(current_building+"Ew", current_building+"Ewr", (113 - 92) / v_l)

    graph.add_edge(current_building+"A", current_building+"B", (75-60)/v_l, bidirectional=False)
    graph.add_edge(current_building+"Br", current_building+"Ar", (75-60)/v_l, bidirectional=False)

    graph.add_edge(current_building+"B", current_building+"Er", (77-75)/v_l, bidirectional=False)
    graph.add_edge(current_building+"E", current_building+"Br", (77-75)/v_l, bidirectional=False)

    graph.add_edge(current_building+"Stair1_1", current_building+"E",  (93-77)/v_l, bidirectional=False)
    graph.add_edge(current_building+"Stair1_1", current_building+"Stair1_2", (122-93)/v_l, bidirectional=False)
    graph.add_edge(current_building+"Cr", current_building+"Stair1_2",  (143-122)/v_l, bidirectional=False)

    graph.add_edge(current_building+"Er", current_building+"Stair1_2",  (122-77+113-92)/v_l, bidirectional=False)
    graph.add_edge(current_building+"Er", current_building+"C", (153-77)/v_l, bidirectional=False)

    graph.add_edge(current_building+"F", current_building+"Cr", (200-143)/v_l, bidirectional=False)
    graph.add_edge(current_building+"C", current_building+"Fr", (200-143)/v_l, bidirectional=False)


    graph.add_edge(current_building+"Fr", current_building+"D", (220-200)/v_l, bidirectional=False)
    graph.add_edge(current_building+"Dr", current_building+"F", (220-200)/v_l, bidirectional=False)

    graph.add_edge(current_building+"G", current_building+"Dr", (293-220)/v_l, bidirectional=False)
    graph.add_edge(current_building+"D", current_building+"Gr", (293-220)/v_l, bidirectional=False)

    graph.add_edge(current_building+"Ew", current_building+"G", (380-293)/v_l, bidirectional=False)
    graph.add_edge(current_building+"Stair2_1", current_building+"G",  (380-293)/v_l, bidirectional=False)
    graph.add_edge(current_building+"Ew", current_building+"Stair2_2", stair_length_2/v_l)

    graph.add_edge(current_building+"Gr", current_building+"Ewr", (380-293)/v_l,bidirectional=False)
    graph.add_edge(current_building+"Gr", current_building+"Stair2_2", (380-293)/v_l,bidirectional=False)


    graph.add_edge(current_building+"Ew", current_building+"E1", (150-110)/v_l)
    graph.add_edge(current_building+"E1", current_building+"E2", (410-380)/v_l)

    graph.add_edge(current_building+"Right_1",current_building+"Ew", (433-380)/v_l, bidirectional=False)
    graph.add_edge(current_building+"Ewr", current_building+"Right_2", (433-380)/v_l, bidirectional=False)
    graph.add_edge(current_building+"Right_2", current_building+"Right_1", (113-92)/v_l)

    #Building 1 first floor to second floor
    if up_building != "":
        graph.add_edge(up_building + "Stair1_1", current_building + "Stair1_1", stair_length_1 / v_s,
                       bidirectional=False)
        graph.add_edge(current_building + "Stair1_2", up_building + "Stair1_2", stair_length_1 / v_s,
                       bidirectional=False)
        graph.add_edge(up_building + "Stair2_1", current_building + "Stair2_1", stair_length_2 / v_s,
                       bidirectional=False)
        graph.add_edge(current_building + "Stair2_2", up_building + "Stair2_2", stair_length_2 / v_s,
                       bidirectional=False)
    # 一层 一号楼 几号楼梯 左1右2
    if right_building != "":
        graph.add_edge(right_building + "Left_1", current_building + "Right_1", 200 / v_l, bidirectional=False)
        graph.add_edge(current_building + "Right_2", right_building + "Left_2", 200 / v_l, bidirectional=False)
        graph.add_edge(right_building + "Left_1", right_building + "Left_2", 200 / v_l)
    # Building1-2

    return graph


def add_floor1(graph,stair_length_1,stair_length_2,v_l,v_s):
    # Floor 1
    # Building 1
    current_building = "1_1_"
    up_building = "2_1_"
    right_building = "1_2_"
    add_a_building_layer(graph,current_building,up_building,right_building,stair_length_1,stair_length_2,v_l,v_s)


    current_building = "1_2_"
    up_building = "2_2_"
    right_building = "1_3_"
    # Building 2################################3
    add_a_building_layer(graph,current_building,up_building,right_building,stair_length_1,stair_length_2,v_l,v_s)


    current_building = "1_3_"
    up_building = "2_3_"
    right_building = ""
    # Building 3
    add_a_building_layer(graph,current_building,up_building,right_building,stair_length_1,stair_length_2,v_l,v_s)

    # --- Print connectivity summary ---
    print("=== Graph Connectivity Summary ===")
    for node, neighbors in graph.edges.items():
        print(f"{node}:")
        for n, w in neighbors:
            print(f"   → {n} (time={w:.2f}s)")
    print(f"\nTotal Nodes: {len(graph.edges)}")
    return graph