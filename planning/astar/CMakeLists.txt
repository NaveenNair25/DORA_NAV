cmake_minimum_required(VERSION 3.10)
project(PathPlanning)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(yaml-cpp REQUIRED)

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${YAML_CPP_INCLUDE_DIR}
    $ENV{HOME}/dora/apis/c/node/ #dora的头文件路径 node_api.h
    $ENV{HOME}/dora/apis/c/operator/
    /usr/local/include
)

# Source files for library
set(SOURCES
    src/map_loader.cpp
    src/astar.cpp
    src/dijkstra.cpp
)

# Create library
add_library(path_planning_lib ${SOURCES})
target_link_libraries(path_planning_lib ${YAML_CPP_LIBRARIES})

# Test executable
add_executable(test_planning src/test_planning.cpp)
target_link_libraries(test_planning path_planning_lib)

# Visualization executable (with rerun)
add_executable(visualize_planning src/visualize_planning.cpp)
target_link_libraries(visualize_planning path_planning_lib rerun_sdk arrow)

# DORA node executable
add_executable(dora_planning_node src/dora_planning_node.cpp)
target_link_libraries(dora_planning_node 
				path_planning_lib
				$ENV{HOME}/dora/target/release/libdora_node_api_c.a
)
