#!/bin/bash

echo "=== 路径规划项目编译脚本 ==="
echo ""

# 检查依赖
echo "检查依赖..."
if ! command -v cmake &> /dev/null; then
    echo "错误: 未找到cmake，请先安装"
    exit 1
fi

if ! command -v g++ &> /dev/null; then
    echo "错误: 未找到g++，请先安装"
    exit 1
fi

# 创建build目录
if [ -d "build" ]; then
    echo "清理旧的build目录..."
    rm -rf build
fi

echo "创建build目录..."
mkdir build
cd build

# 运行CMake
echo "运行CMake配置..."
cmake .. || {
    echo "CMake配置失败！"
    exit 1
}

# 编译
echo "开始编译..."
make -j$(nproc) || {
    echo "编译失败！"
    exit 1
}

echo ""
echo "=== 编译成功！ ==="
echo ""
echo "可执行文件："
echo "  - ./build/test_planning          (测试节点)"
echo "  - ./build/visualize_planning     (可视化节点)"
echo "  - ./build/dora_planning_node     (DORA节点)"
echo ""
echo "运行测试："
echo "  cd build && ./test_planning"
echo ""
echo "运行DORA节点："
echo "  cd build && ./dora_planning_node"
echo ""
