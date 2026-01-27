"""
API 接口测试脚本

测试流程：
1. GET / - 根路径
2. POST /initialize-map - 初始化地图
3. POST /schedule-tasks - 批量任务调度
4. GET /robot-status - 查询机器人状态
"""

import requests
import json
import os

BASE_URL = "http://localhost:8000"

def test_root():
    """测试根路径"""
    print("\n=== Test 1: Root endpoint ===")
    response = requests.get(f"{BASE_URL}/")
    print(f"Status: {response.status_code}")
    print(f"Response: {response.json()}")
    assert response.status_code == 200
    assert "message" in response.json()
    print("[PASS]")

def test_initialize_map():
    """测试初始化地图"""
    print("\n=== Test 2: Initialize map ===")

    # 使用已有的 test_campus 数据
    campus_name = "test_campus"
    yaml_dir = "D:/code/4_28/src/core/data/test_campus/yaml"

    # 获取 YAML 文件
    yaml_files = []
    for filename in os.listdir(yaml_dir):
        if filename.endswith(".yaml"):
            filepath = os.path.join(yaml_dir, filename)
            yaml_files.append(("yaml_files", (filename, open(filepath, "rb"), "application/octet-stream")))

    # 表单数据
    data = {
        "campus_name": campus_name,
        "robots": json.dumps({
            "robots": [
                {"rid": 0, "skill": "dog", "position": "4_1_p1", "campus_name": campus_name},
                {"rid": 1, "skill": "dog", "position": "4_1_p1", "campus_name": campus_name},
                {"rid": 2, "skill": "human", "position": "4_1_p1", "campus_name": campus_name},
                {"rid": 3, "skill": "human", "position": "4_1_p1", "campus_name": campus_name}
            ]
        })
    }

    try:
        response = requests.post(
            f"{BASE_URL}/initialize-map",
            files=yaml_files,
            data=data
        )
        print(f"Status: {response.status_code}")
        result = response.json()
        print(f"Response: {result}")

        # 关闭文件
        for _, (_, file_obj, _) in yaml_files:
            file_obj.close()

        if response.status_code == 200:
            print("[PASS]")
            return True
        else:
            print("[FAIL]")
            return False
    except Exception as e:
        print(f"Error: {e}")
        # 关闭文件
        for _, (_, file_obj, _) in yaml_files:
            try:
                file_obj.close()
            except:
                pass
        return False

def test_schedule_tasks():
    """测试批量任务调度"""
    print("\n=== Test 3: Schedule tasks ===")

    # 使用实际存在的节点名称
    batch_input = {
        "tasks": [
            {
                "task_id": 1,
                "skill": "dog",
                "priority": 1,
                "start_position": "4_1_p1",
                "target_position": "5_1_p1"
            },
            {
                "task_id": 2,
                "skill": "human",
                "priority": 2,
                "start_position": "4_1_p1",
                "target_position": "4_1_p7"
            }
        ]
    }

    try:
        response = requests.post(
            f"{BASE_URL}/schedule-tasks",
            json=batch_input
        )
        print(f"Status: {response.status_code}")

        if response.status_code == 200:
            results = response.json()
            print(f"Response: {json.dumps(results, indent=2, ensure_ascii=False)}")
            for task in results:
                print(f"\n  Task {task['task_id']}:")
                print(f"    Robot: {task['robot_id']}")
                print(f"    Path: {' -> '.join(task['path'][:5])}... ({len(task['path'])} nodes)")
                print(f"    Estimated time: {task['estimated_time']:.2f}s")
            print("[PASS]")
            return True
        else:
            print(f"Response: {response.text}")
            print("[FAIL]")
            return False
    except Exception as e:
        print(f"Error: {e}")
        return False

def test_robot_status():
    """测试查询机器人状态"""
    print("\n=== Test 4: Robot status ===")

    try:
        response = requests.get(f"{BASE_URL}/robot-status")
        print(f"Status: {response.status_code}")

        if response.status_code == 200:
            data = response.json()
            print(f"Response: {json.dumps(data, indent=2, ensure_ascii=False)[:500]}...")
            print("[PASS]")
            return True
        else:
            print(f"Response: {response.text}")
            print("[FAIL]")
            return False
    except Exception as e:
        print(f"Error: {e}")
        return False

def main():
    print("=" * 60)
    print("API Interface Test")
    print("=" * 60)

    results = []

    # Test 1
    results.append(("Root", test_root()))

    # Test 2
    results.append(("Initialize", test_initialize_map()))

    # Test 3
    results.append(("Schedule", test_schedule_tasks()))

    # Test 4
    results.append(("Status", test_robot_status()))

    # Summary
    print("\n" + "=" * 60)
    print("Test Summary")
    print("=" * 60)
    for name, passed in results:
        status = "[PASS]" if passed else "[FAIL]"
        print(f"{name}: {status}")

    passed_count = sum(1 for _, p in results if p)
    print(f"\nTotal: {passed_count}/{len(results)} passed")

if __name__ == "__main__":
    main()