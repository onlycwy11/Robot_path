import robot_path

# 如果 scheduler 已经初始化，直接访问
status = robot_path.get_robot_status_real_time()
print(status)
