#!/usr/bin/env python3
"""
使用Rerun可视化路径规划结果
需要安装: pip install rerun-sdk pillow pyyaml
"""

import rerun as rr
import numpy as np
from PIL import Image
import yaml
import sys

def load_map(yaml_file):
    """加载地图文件"""
    with open(yaml_file, 'r') as f:
        map_info = yaml.safe_load(f)

    # 获取pgm文件路径
    import os
    map_dir = os.path.dirname(yaml_file)
    pgm_file = os.path.join(map_dir, map_info['image'])

    # 加载PGM图像
    img = Image.open(pgm_file)
    map_data = np.array(img)

    return map_data, map_info

def load_path(path_file):
    """加载路径文件"""
    points = []
    with open(path_file, 'r') as f:
        for line in f:
            line = line.strip()
            if line:
                x, y = map(int, line.split(','))
                points.append([x, y])
    return np.array(points)

def main():
    if len(sys.argv) < 2:
        print("Usage: python visualize.py <map_yaml_file>")
        sys.exit(1)

    map_file = sys.argv[1]

    # 初始化Rerun
    rr.init("path_planning_visualization", spawn=True)

    # 加载地图
    print(f"Loading map: {map_file}")
    map_data, map_info = load_map(map_file)
    print(f"Map size: {map_data.shape}")

    # 记录地图图像
    rr.log("world/map", rr.Image(map_data))

    # 加载并可视化A*路径
    try:
        astar_path = load_path("astar_path.txt")
        print(f"A* path loaded: {len(astar_path)} points")
        rr.log("world/astar_path",
               rr.LineStrips2D(astar_path),
               rr.AnyValues(color=[255, 0, 0, 255]))
    except FileNotFoundError:
        print("A* path file not found")

    # 加载并可视化Dijkstra路径
    try:
        dijkstra_path = load_path("dijkstra_path.txt")
        print(f"Dijkstra path loaded: {len(dijkstra_path)} points")
        rr.log("world/dijkstra_path",
               rr.LineStrips2D(dijkstra_path),
               rr.AnyValues(color=[0, 255, 0, 255]))
    except FileNotFoundError:
        print("Dijkstra path file not found")

    print("\nVisualization complete! Check the Rerun viewer.")

if __name__ == "__main__":
    main()

