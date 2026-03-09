#!/usr/bin/env python3
"""
DORA Path Visualization Node
接收DORA路径规划节点发布的轨迹，使用Rerun进行可视化
"""

import json
import numpy as np
from PIL import Image
import yaml
import os

# DORA imports
from dora import Node

# Rerun import
import rerun as rr


def load_map(yaml_file):
    """加载地图文件"""
    with open(yaml_file, 'r') as f:
        map_info = yaml.safe_load(f)

    map_dir = os.path.dirname(yaml_file)
    pgm_file = os.path.join(map_dir, map_info['image'])

    img = Image.open(pgm_file)
    map_data = np.array(img)

    return map_data, map_info


def draw_circle(rgb, cx, cy, radius, color):
    """在RGB图像上绘制实心圆"""
    height, width = rgb.shape[:2]
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            if dx * dx + dy * dy <= radius * radius:
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < width and 0 <= ny < height:
                    rgb[ny, nx] = color


def draw_path_on_map(base_map, path_points, path_color, start_color, goal_color):
    """将路径融合到地图图像中，返回RGB图像"""
    # 灰度图转RGB
    rgb = np.stack([base_map, base_map, base_map], axis=-1).copy()

    height, width = rgb.shape[:2]

    # 绘制路径像素
    for p in path_points:
        x, y = int(p[0]), int(p[1])
        if 0 <= x < width and 0 <= y < height:
            rgb[y, x] = path_color

    # 绘制起点（红色圆圈）
    if path_points:
        draw_circle(rgb, int(path_points[0][0]), int(path_points[0][1]), 5, start_color)

    # 绘制终点（绿色圆圈）
    if path_points:
        draw_circle(rgb, int(path_points[-1][0]), int(path_points[-1][1]), 5, goal_color)

    return rgb


def main():
    rr.init("dora_path_planning_visualization", spawn=True)

    map_yaml_path = os.getenv("MAP_YAML_PATH", "map/test_map.yaml")
    print(f"Loading map: {map_yaml_path}")

    try:
        base_map, map_info = load_map(map_yaml_path)
        print(f"Map loaded: {base_map.shape}")
        rr.log("world/map", rr.Image(base_map))
    except Exception as e:
        print(f"Failed to load map: {e}")
        return

    node = Node()
    print("DORA Path Visualization Node started")
    print("Waiting for path data...")

    for event in node:
        event_type = event["type"]

        if event_type == "INPUT" and event["id"] == "path":
            data = event["value"]
            try:
                # DORA从C++节点接收的数据是PyArrow uint8数组，需要先转换为bytes
                if hasattr(data, 'to_pylist'):
                    data_str = bytes(data.to_pylist()).decode('utf-8')
                elif isinstance(data, bytes):
                    data_str = data.decode('utf-8')
                else:
                    data_str = str(data)
                path_data = json.loads(data_str)

                print(f"\n=== Received Path ===")
                print(f"Algorithm: {path_data['algorithm']}")
                print(f"Path found: {path_data['path_found']}")
                print(f"Path length: {path_data['path_length']:.2f} grid units")
                print(f"Compute time: {path_data['compute_time_ms']:.2f} ms")
                print(f"Waypoints: {path_data['num_waypoints']}")

                if path_data['path_found']:
                    waypoints = path_data['waypoints']
                    points = [[p['x'], p['y']] for p in waypoints]

                    if path_data['algorithm'] == 'astar':
                        path_color = [0, 0, 255]    # 蓝色
                        log_name = "world/map_astar"
                    else:
                        path_color = [0, 255, 255]  # 青色
                        log_name = "world/map_dijkstra"

                    map_with_path = draw_path_on_map(
                        base_map, points,
                        path_color=path_color,
                        start_color=[255, 0, 0],    # 红色起点
                        goal_color=[0, 255, 0]      # 绿色终点
                    )
                    rr.log(log_name, rr.Image(map_with_path))
                    print(f"Path fused into map and logged to {log_name}")
                else:
                    print("No valid path found!")

            except json.JSONDecodeError as e:
                print(f"JSON decode error: {e}")
            except Exception as e:
                print(f"Error processing path data: {e}")

        elif event_type == "STOP":
            print("Received stop event")
            break

    print("DORA Path Visualization Node stopped")


if __name__ == "__main__":
    main()
