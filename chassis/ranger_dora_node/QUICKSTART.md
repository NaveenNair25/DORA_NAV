# Ranger Mini DORA节点

## 📦 项目文件清单

```
ranger_dora_node/
├── src/
│   └── ranger_dora_node.cc          # 主程序源代码
├── CMakeLists.txt                    # CMake编译配置
├── ranger_miniv3_dataflow.yml        # DORA数据流配置
├── build.sh                          # 编译脚本
├── keyboard_control.py               # 键盘控制测试脚本
├── README.md                         # 详细使用文档
├── CHECKLIST.md                      # 开发检查清单
├── INTERFACE_ANALYSIS.md             # 接口分析文档
└── QUICKSTART.md                     # 本文件
```



## ⚠️ 常见问题快速解决

### 问题1: 编译失败 - 找不到ugv_sdk

**解决方案**：修改CMakeLists.txt中的路径
```cmake
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/../ugv_sdk/include  # 改为实际路径
)
```

### 问题2: CAN连接失败

**解决方案**：
```bash
# 检查CAN接口状态
ip link show can0

# 如果down，重新启动
sudo ip link set can0 up

# 检查权限
sudo chmod 666 /dev/can0  # 或添加用户到dialout组
```

### 问题3: DORA启动失败

**解决方案**：
```bash
# 检查dataflow配置
dora check ranger_miniv3_dataflow.yml

# 查看详细错误
dora start ranger_miniv3_dataflow.yml --verbose
```

### 问题4: 机器人不响应命令

**检查清单**：
1. CAN总线是否正常？`candump can0`
2. 节点是否运行？`dora list`
3. 命令是否发送？检查keyboard节点日志
4. 机器人是否上电？检查电源和急停开关

## 📊 验证节点工作正常

### 检查1: 查看输出数据

```bash
# 监听Odometry输出
dora listen ranger_miniv3_node/Odometry

# 监听SystemState输出
dora listen ranger_miniv3_node/SystemState
```

### 检查2: 发送测试命令

```bash
# 使用dora发送命令（需要安装dora CLI工具）
echo '{"linear":{"x":0.1,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}' | \
  dora send ranger_miniv3_node/CmdVelTwist
```

### 检查3: 监控CAN通信

```bash
# 安装can-utils
sudo apt-get install can-utils

# 监听CAN消息
candump can0
```

应该看到类似的输出：
```
can0  181   [8]  00 00 00 00 00 00 00 00
can0  281   [8]  64 00 00 00 00 00 00 00
```

## 🎯 下一步

1. **阅读详细文档**：查看README.md了解完整功能
2. **检查清单**：查看CHECKLIST.md确认所有配置正确
3. **接口分析**：查看INTERFACE_ANALYSIS.md了解技术细节
4. **自定义配置**：根据需要修改dataflow.yml和环境变量
 

如果看到以下输出，说明节点运行正常：
```
========================================
Ranger Mini V3 DORA Node
========================================
Configuration:
  CAN Port: can0
  Update Rate: 100 Hz
========================================
Connected to CAN port: can0
Commanded mode enabled
DORA context initialized
Starting main loop...
```
 
