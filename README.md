# Handex-Robots: ROS2智能机器人控制系统

![ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange.svg)
![ros2](https://img.shields.io/badge/ros2-humble-blue.svg)
![python](https://img.shields.io/badge/python-3.10-green.svg)

本项目是基于ROS2 Humble的多机器人协同控制系统，集成了移动机器人、机械臂、激光雷达等多种硬件设备，实现了自主导航、路径规划、任务调度等功能。

## 📋 项目概述

该系统采用Master-Slave分布式架构，支持多机器人协同工作：

- **Master节点**: 负责路径规划、地图构建、全局任务调度
- **Slave节点**: 负责局部任务执行、设备控制

### 主要功能特性

- 🤖 多机器人协同控制
- 🗺️ SLAM建图与定位
- 🎯 智能路径规划（A*算法 + MPC控制）
- 🦾 机械臂控制（Piper机械臂支持）
- 🎮 主从遥操作（Viola/Violin主臂控制Piper从臂）
- 🔄 数据采集和回放（基于LeRobot框架）
- 📡 激光雷达集成（RPLidar支持）
- 🚗 移动平台控制（Tracer机器人支持）
- 📱 任务调度系统

## 🏗️ 系统架构

```
Handex-Robots/
├── Master/                 # 主控节点
│   ├── src/               # 源码目录
│   │   ├── algorithm/     # 算法包
│   │   ├── piper_ros/     # 机械臂控制
│   │   ├── rplidar_ros/   # 激光雷达
│   │   └── tracer_ros2/   # 移动平台
│   ├── install/           # 编译输出
│   ├── map/              # 地图文件
│   └── script/           # 启动脚本
└── Slave/                 # 从控节点
    ├── dispatch_pkg/      # 调度包
    ├── task_handler/      # 任务处理
    └── lerobot_piper/     # 主从遥操作模块
        ├── viola_piper_teleop.py      # 基础遥操作
        ├── viola_piper_advanced.py   # 高级遥操作
        ├── README_Viola_Piper_Teleop.md  # 遥操作文档
        └── VIOLA_PIPER_LEROBOT_GUIDE.md  # LeRobot集成指南
```

## 🔧 硬件支持

### 移动平台
- **Tracer Robot**: 全地形移动机器人
- **Tracer Mini**: 小型移动平台

### 机械臂
- **Piper机械臂**: 6自由度工业机械臂
- 支持带夹爪和无夹爪两种配置
- **Viola/Violin机械臂**: 作为主臂进行遥操作控制
- **主从遥操作**: 支持Viola/Violin主臂控制Piper从臂

### 传感器
- **RPLidar系列**: A1/A2/A3/S1/S2/S3/T1/C1等多款型号
- **CAN总线**: 设备通信
- **USB转CAN**: 通信适配器

## 🚀 快速开始

### 环境要求

```bash
# 系统要求
Ubuntu 22.04 LTS
ROS2 Humble
Python >= 3.10
```

### 依赖安装

```bash
# 安装ROS2控制相关包
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-ros2-controllers
sudo apt install ros-humble-controller-manager

# 安装CAN工具
sudo apt update && sudo apt install can-utils ethtool iproute2

# 安装Python依赖
pip3 install python-can scipy piper_sdk

# 确保python-can版本 >= 4.3.1
pip3 install --upgrade python-can

# 安装主从遥操作依赖
pip3 install numpy torch h5py matplotlib lerobot
```

### 编译项目

```bash
# 创建工作空间
mkdir -p ~/handex_ws/src
cd ~/handex_ws/src

# 克隆项目
git clone <your-repo-url> handex-robots

# 编译
cd ~/handex_ws
colcon build

# 设置环境变量
source install/setup.bash
```

### CAN总线配置

```bash
# 查找CAN设备
cd Master/src/piper_ros
bash find_all_can_port.sh

# 启动CAN接口
cd ~/handex_ws/src/handex-robots/Master/script
bash bringup_can2usb_500k.bash
```

## 🎮 运行系统

### 1. 启动移动平台

```bash
# 标准Tracer
ros2 launch tracer_base tracer_base.launch.py

# 或者Tracer Mini
ros2 launch tracer_base tracer_mini_base.launch.py
```

### 2. 启动机械臂

```bash
# 带夹爪版本
ros2 launch piper_with_gripper_moveit piper_moveit.launch.py

# 无夹爪版本  
ros2 launch piper_no_gripper_moveit piper_moveit.launch.py
```

### 3. 启动激光雷达

```bash
# 根据你的雷达型号选择
ros2 launch rplidar_ros rplidar_s1_launch.py  # S1型号
ros2 launch rplidar_ros rplidar_a3_launch.py  # A3型号
```

### 4. 启动导航系统

```bash
# 路径规划和MPC控制
ros2 launch my_pkg APF_MPC.launch.py GOAL_X:=2.0 GOAL_Y:=3.0
```

### 5. 启动任务调度

```bash
# Master调度节点
ros2 run dispatch_pkg dispatch_car

# Slave任务处理
ros2 run task_handler task_handler_node
```

### 6. 主从机械臂遥操作

```bash
# 进入遥操作模块目录
cd Slave/lerobot_piper

# 检查设备连接
python lerobot/scripts/find_viola_piper_ports.py

# 基础遥操作（Viola主臂控制Piper从臂）
python viola_piper_teleop.py

# 高级遥操作（包含力反馈和可视化）
python viola_piper_advanced.py

# 使用LeRobot框架进行数据采集
python viola_piper_control_robot.py \
    --control.type=record \
    --control.fps=30 \
    --control.num_episodes=50 \
    --control.episode_time_s=30
```

## 🛠️ 核心算法模块

### 路径规划算法
- **A*算法**: 全局路径规划
- **人工势场法(APF)**: 局部避障
- **MPC控制器**: 轨迹跟踪控制

### SLAM建图
- **slam_toolbox**: 实时建图定位
- **grid_map**: 栅格地图表示
- **配置文件**: `Master/script/my_slam_config.yaml`

### 运动控制
- **差分驱动模型**: 移动平台控制
- **MoveIt!**: 机械臂运动规划
- **CAN总线通信**: 底层硬件接口
- **主从遥操作**: Viola/Violin主臂控制Piper从臂
- **关节映射算法**: 主从臂关节空间映射
- **力反馈系统**: 实时力反馈控制

## 📡 通信接口

### 主要话题

| 话题名称 | 消息类型 | 描述 |
|---------|---------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | 速度控制命令 |
| `/odom` | `nav_msgs/Odometry` | 里程计信息 |
| `/scan` | `sensor_msgs/LaserScan` | 激光雷达数据 |
| `/goal_pose` | `geometry_msgs/PoseStamped` | 目标位置 |
| `/task` | `std_msgs/Int8` | 任务指令 |
| `/car_state` | `std_msgs/Int8` | 机器人状态 |
| `/arm_state` | `std_msgs/Int8` | 机械臂状态 |
| `/leader_joints` | `sensor_msgs/JointState` | 主臂关节状态 |
| `/follower_joints` | `sensor_msgs/JointState` | 从臂关节状态 |
| `/force_feedback` | `geometry_msgs/WrenchStamped` | 力反馈信息 |

### 服务接口

| 服务名称 | 服务类型 | 描述 |
|---------|---------|------|
| `/piper_msgs/srv/*` | 自定义服务 | 机械臂控制服务 |
| `/tracer_msgs/srv/*` | 自定义服务 | 移动平台服务 |

## 🎯 任务调度系统

### 任务类型定义

```cpp
enum TaskType {
    TASK_NAVIGATION = 10,        // 导航任务
    TASK_ARM_CONTROL = 15,       // 机械臂控制
    TASK_TELEOPERATION = 20,     // 主从遥操作
    TASK_DATA_COLLECTION = 25,   // 数据采集
    TASK_IDLE = 0,              // 空闲状态
    TASK_UNKNOWN = 255          // 未知任务
};
```

### 状态机制

```cpp
enum RobotState {
    STATE_IDLE = 0,         // 空闲
    STATE_BUSY = 1,         // 忙碌
    STATE_ERROR = 2         // 错误
};
```

## 📊 可视化界面

系统集成了RViz2可视化工具：

```bash
# 启动RViz可视化
ros2 run rviz2 rviz2 -d Master/src/algorithm/my_pkg/rviz/my_rviz.rviz
```

可视化内容包括：
- 🗺️ 实时地图显示
- 🤖 机器人位姿
- 📍 路径规划结果
- 🎯 目标点设置
- 📡 传感器数据
- 🦾 主从臂关节状态对比
- 💪 力反馈数据可视化
- 📊 关节轨迹记录

## 🔐 安全注意事项

⚠️ **重要安全提醒**:

1. **急停准备**: 始终准备好遥控器接管控制
2. **测试环境**: 在安全的测试环境中运行
3. **权限检查**: 确保CAN设备有足够的访问权限
4. **速度限制**: 调试时使用较低的运动速度

## 🐛 故障排除

### 常见问题

1. **CAN设备连接失败**
   ```bash
   # 检查设备权限
   sudo chmod 666 /dev/ttyUSB*
   # 重新加载驱动
   sudo modprobe gs_usb
   ```

2. **ROS节点启动失败**
   ```bash
   # 检查环境变量
   source /opt/ros/humble/setup.bash
   source ~/handex_ws/install/setup.bash
   ```

3. **激光雷达无数据**
   ```bash
   # 检查设备连接
   ls /dev/ttyUSB*
   # 检查权限
   sudo chmod 777 /dev/ttyUSB0
   ```

4. **主从遥操作连接失败**
   ```bash
   # 检查Viola机械臂串口
   ls /dev/ttyUSB*
   sudo chmod 666 /dev/ttyUSB0
   
   # 检查Piper机械臂CAN接口
   ip link show can0
   sudo ip link set can0 up type can bitrate 1000000
   ```

5. **遥操作延迟过高**
   ```bash
   # 优化系统性能
   sudo sysctl -w net.core.rmem_max=134217728
   sudo sysctl -w net.core.wmem_max=134217728
   
   # 使用实时内核（可选）
   sudo apt install linux-lowlatency
   ```

### 调试工具

```bash
# 查看话题列表
ros2 topic list

# 监听话题数据
ros2 topic echo /scan

# 查看节点状态
ros2 node list

# 检查参数配置
ros2 param list

# 遥操作调试
# 监听主臂关节状态
ros2 topic echo /leader_joints

# 监听从臂关节状态  
ros2 topic echo /follower_joints

# 检查力反馈数据
ros2 topic echo /force_feedback

# 测试遥操作连接
python Slave/lerobot_piper/test_viola_piper_teleop.py
```

## 🤝 贡献指南

欢迎提交Issue和Pull Request来改进项目。

### 开发规范

1. 遵循ROS2编码规范
2. 添加适当的注释和文档
3. 测试新功能的兼容性
4. 更新相应的配置文件

## 📄 许可证

本项目采用多种开源许可证：
- 机械臂部分: Apache-2.0
- 移动平台部分: BSD
- 算法部分: TODO(待更新)

## 📞 联系方式

- 维护者: humble2204
- 邮箱: 95029893+Rigelllll@users.noreply.github.com

## 🙏 致谢

感谢以下开源项目的支持：
- [ROS2](https://ros.org/)
- [MoveIt2](https://moveit.ai/)
- [LeRobot](https://github.com/huggingface/lerobot)
- [AgileX Robotics](https://github.com/agilexrobotics)
- [SLAMTEC](https://www.slamtec.com/)

---

*最后更新: 2025年7月*