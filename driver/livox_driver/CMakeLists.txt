cmake_minimum_required(VERSION 3.10)
project(livox_dora_driver)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

#========================
#  dora
#========================
set(DORA_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/dora/include)

set(DORA_NODE_API_LIB 
${CMAKE_SOURCE_DIR}/dora/lib/libdora_node_api_c.a
    CACHE FILEPATH "Path to libdora_node_api_c.a"
)

# Add definitions
add_definitions(-DBUILDING_DORA)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/include
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty
    ${DORA_INCLUDE_DIR}
    /usr/local/include
)

# Find required packages
find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)

# Find nlohmann/json (assuming it's installed system-wide)
find_package(nlohmann_json REQUIRED)

# Find Livox SDK (assuming it's installed)
find_library(LIVOX_SDK_LIBRARY
    NAMES livox_lidar_sdk_static
    PATHS /usr/local/lib /usr/lib
    REQUIRED
)

link_directories(/usr/local/lib)

# Source files for the Dora driver
set(DORA_DRIVER_SOURCES
    src/livox_dora_driver.cpp
    src/dora_node.cpp
    src/lddc_dora.cpp
    src/lds.cpp
    src/lds_lidar.cpp
    src/comm/cache_index.cpp
    src/comm/comm.cpp
    src/comm/ldq.cpp
    src/comm/lidar_imu_data_queue.cpp
    src/comm/pub_handler.cpp
    src/comm/semaphore.cpp
    src/call_back/lidar_common_callback.cpp
    src/call_back/livox_lidar_callback.cpp
    src/parse_cfg_file/parse_cfg_file.cpp
    src/parse_cfg_file/parse_livox_lidar_cfg.cpp
)

# Create executable
add_executable(${PROJECT_NAME} ${DORA_DRIVER_SOURCES})

# Link libraries
target_link_libraries(${PROJECT_NAME}
    ${LIVOX_SDK_LIBRARY}
    nlohmann_json::nlohmann_json
    Threads::Threads
    ${DORA_NODE_API_LIB}
    liblivox_lidar_sdk_static.a
)

# Platform-specific libraries
if(WIN32)
    target_link_libraries(${PROJECT_NAME} ws2_32)
elseif(UNIX)
    target_link_libraries(${PROJECT_NAME} pthread)
endif()

# Install target
install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION bin
)

# Install config files
install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
    FILES_MATCHING PATTERN "*.json"
)

# Print build information
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Compiler: ${CMAKE_CXX_COMPILER}")
message(STATUS "Livox SDK library: ${LIVOX_SDK_LIBRARY}")