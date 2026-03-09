#!/usr/bin/env python3
"""
DORA Test Input Node
发送起点和终点坐标给路径规划节点
"""

import json
import time
from dora import Node


def main():
    node = Node()

    print("DORA Test Input Node started")

    # 定义测试用例
    test_cases = [
        {
            "name": "Test Case 1",
            "start": {"x": 50, "y": 50},
            "goal": {"x": 400, "y": 400}
        },
        {
            "name": "Test Case 2",
            "start": {"x": 100, "y": 100},
            "goal": {"x": 500, "y": 500}
        },
        {
            "name": "Test Case 3",
            "start": {"x": 200, "y": 200},
            "goal": {"x": 300, "y": 300}
        }
    ]

    test_index = 0

    for event in node:
        event_type = event["type"]

        if event_type == "INPUT":
            event_id = event["id"]

            if event_id == "tick":
                # 每次tick发送一个测试用例
                if test_index < len(test_cases):
                    test_case = test_cases[test_index]

                    print(f"\n=== {test_case['name']} ===")
                    print(f"Start: {test_case['start']}")
                    print(f"Goal: {test_case['goal']}")

                    # 发送起点
                    start_json = json.dumps(test_case['start'])
                    node.send_output("start_point", start_json.encode('utf-8'))
                    print("Sent start_point")

                    # 稍微延迟
                    time.sleep(0.1)

                    # 发送终点
                    goal_json = json.dumps(test_case['goal'])
                    node.send_output("goal_point", goal_json.encode('utf-8'))
                    print("Sent goal_point")

                    test_index += 1

                    # 等待一段时间再发送下一个测试用例
                    time.sleep(5)
                else:
                    print("\nAll test cases completed")
                    break

        elif event_type == "STOP":
            print("Received stop event")
            break

    print("DORA Test Input Node stopped")


if __name__ == "__main__":
    main()
