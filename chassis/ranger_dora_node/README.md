# Ranger Mini V3 DORA Node

Ranger Mini V3 DORA节点实现，基于Ranger Mini V3 ROS2节点代码修改，用于控制AgileX Ranger Mini V3机器人。

## 1 功能特性

- ✅ 支持Ranger Mini V3机器人
- ✅ 通过CAN总线与ugv_sdk通信
- ✅ 支持4种运动模式：
  - 双阿克曼转向 (Dual Ackerman)
  - 平行模式 (Parallel/Omnidirectional)
  - 原地旋转 (Spinning)
  - 侧滑模式 (Side Slip - 仅V1支持)
- ✅ 实时里程计计算
- ✅ 完整的状态反馈（系统状态、执行器状态、电池状态）
- ✅ JSON格式数据传输
- ✅ 100Hz高频率更新

### 1.1 输入输出接口

#### 输入
- **CmdVelTwist**: 速度命令 (JSON格式)
  ```json
  {
    "linear": {"x": 0.5, "y": 0.0, "z": 0.0},
    "angular": {"x": 0.0, "y": 0.0, "z": 0.2}
  }
  ```


#### 输出
- **Odometry**: 里程计数据（位置、姿态、速度）
- **SystemState**: 系统状态（车辆状态、控制模式、错误码、电池电压、运动模式）
- **MotionState**: 当前运动模式
- **ActuatorState**: 8个执行器的详细状态
- **BatteryState**: BMS电池信息（电压、电流、温度、电量百分比）

### 1.2 依赖项

- CMake >= 3.10
- C++17编译器
- ugv_sdk (AgileX SDK)
- DORA runtime
- nlohmann/json (JSON库)
- Linux系统
- CAN总线接口（can0或can1）
- 已配置的CAN驱动

##  2 节点启动 🚀

### 2.1 步骤1: 检查依赖

```bash
# 检查ugv_sdk
ls ../ugv_sdk/lib/libugv_sdk.so
```

### 2.2 步骤2: 编译

```bash
cd ranger_dora_node
./build.sh
```

如果编译成功，会看到：
```
=========================================
Build successful!
Executable: build/ranger_miniv3_node
=========================================
```

### 2.3 步骤3: 配置CAN总线

```bash
# 检查CAN接口状态
ip link show can0

# 检查权限
sudo chmod 666 /dev/can0  # 或添加用户到dialout组

# 设置CAN总线波特率为500kbps
sudo ip link set can0 type can bitrate 500000

 # 启动CAN接口
sudo ip link set can0 up
 
# 验证CAN接口
ip -details link show can0
```

### 2.4 步骤4: 运行节点

```bash
# 启动DORA
dora up

# 运行dataflow
dora start ranger_miniv3_dataflow.yml
```

### 2.5 步骤5: 测试

在另一个终端：
```bash
# 查看日志
dora logs ranger_miniv3_node

# 或使用键盘控制（如果配置了keyboard节点）
# 按w/s/a/d控制机器人
```

###  2.6 参数配置

在dataflow.yml中配置以下环境变量：

- `CAN_PORT`: CAN接口名称（默认: can0）
- `UPDATE_RATE`: 状态更新频率（默认: 100 Hz）
 

## 3 运动模式说明

节点会根据输入的速度命令自动切换运动模式：

1. **双阿克曼模式** (Dual Ackerman)
   - 条件: linear.y = 0 且转弯半径 >= min_turn_radius
   - 行为: 类似汽车的转向方式

2. **平行模式** (Parallel)
   - 条件: linear.y ≠ 0
   - 行为: 全向移动，可以斜向行驶

3. **原地旋转模式** (Spinning)
   - 条件: linear.y = 0 且转弯半径 < min_turn_radius
   - 行为: 原地旋转

## 机器人参数 (Ranger Mini V3)

```cpp
track = 0.58 m          // 轮距
wheelbase = 0.648 m     // 轴距
max_linear_speed = 1.5 m/s
max_angular_speed = 0.5236 rad/s (30°/s)
max_steer_angle_central = 0.5585 rad (32°)
max_steer_angle_parallel = 0.75 rad (43°)
min_turn_radius = 1.0 m
```

## 4 故障排除

### 4.1 CAN总线连接失败
```
Failed to connect to CAN port: can0
```
解决方案：
1. 检查CAN接口是否已启动: `ip link show can0`
2. 检查波特率设置是否正确（500kbps）
3. 检查硬件连接

### 4.2 权限问题
```
Permission denied
```
解决方案：
```bash
# 添加用户到dialout组
sudo usermod -a -G dialout $USER

# 或使用sudo运行
sudo dora start ranger_miniv3_dataflow.yml
```

### 4.3 编译错误
- 确保已下载ugv_sdk ranger_dora_node中
- 确保已安装DORA runtime
- 检查CMakeLists.txt中的路径设置

### 4.4 与ROS2节点的对比

| 特性 | ROS2节点 | DORA节点 |
|------|---------|---------|
| 通信框架 | ROS2 DDS | DORA |
| 数据格式 | ROS2消息 | JSON |
| 配置方式 | launch文件 | dataflow.yml |
| 依赖 | ROS2 | DORA runtime |
| 性能 | 中等 | 高（更低延迟） |
 
