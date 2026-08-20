# hex_ros_robot_arm
**中文** | [English](README.md)
## 目录

- [1. 包的简介](#1-包的简介)
- [2. 包架构](#2-包架构)
- [3. 话题接口](#3-话题接口)
- [4. 控制模式](#4-控制模式)
- [5. 参数说明](#5-参数说明)
- [6. 依赖关系](#6-依赖关系)
- [7. 快速使用](#7-快速使用)

---

## 1. 包的简介

这是 **HEXFELLOW** 机械臂的 **ROS 驱动包**

- **Archer Y6** — 6 自由度机械臂 + 夹爪，支持 MIT/位置/末端位姿控制
- **Firefly Y6** — 6 自由度机械臂 + 夹爪，与 Archer 结构一致
- **Hello Y6** — 6 自由度只读机械臂（state-only），支持摇杆手柄和 RGB LED

每个节点通过 WebSocket 连接真实控制器，订阅控制指令，发布机械臂实时状态。同时支持 **ROS 1** 和 **ROS 2**。

---

## 2. 包架构

```
hex_ros_robot_arm/
├── config/                        # 参数配置（每个节点独立）
│   ├── ros1/
│   │   ├── archer_params.yaml     #   Archer Y6 ROS 1 参数
│   │   ├── firefly_params.yaml    #   Firefly Y6 ROS 1 参数
│   │   └── hello_params.yaml      #   Hello Y6 ROS 1 参数
│   └── ros2/
│       ├── archer_params.yaml     #   Archer Y6 ROS 2 参数
│       ├── firefly_params.yaml    #   Firefly Y6 ROS 2 参数
│       └── hello_params.yaml      #   Hello Y6 ROS 2 参数
├── launch/                        # 启动文件（每个节点独立）
│   ├── ros1/
│   │   ├── archer.launch          #   Archer Y6 ROS 1 启动
│   │   ├── firefly.launch         #   Firefly Y6 ROS 1 启动
│   │   └── hello.launch           #   Hello Y6 ROS 1 启动
│   └── ros2/
│       ├── archer.launch.py       #   Archer Y6 ROS 2 启动
│       ├── firefly.launch.py      #   Firefly Y6 ROS 2 启动
│       └── hello.launch.py        #   Hello Y6 ROS 2 启动
├── hex_ros_robot_arm/             # 核心代码
│   ├── robot_archer_y6.py         #   Archer Y6 主节点（控制循环 + ROS 接口）
│   ├── robot_firefly_y6.py        #   Firefly Y6 主节点（控制循环 + ROS 接口）
│   ├── robot_hello_y6.py          #   Hello Y6 主节点（只读控制循环 + ROS 接口）
│   ├── test_ctrl.py               #   交互式测试节点（调试工具）
│   ├── utility/                   #   公共 DataInterface（Archer / Firefly 共用）
│   ├── hello_utils/               #   Hello Y6 DataInterface（只读 + 摇杆 + 灯色）
│   └── test_utils/                #   测试用 DataInterface
├── resource/                      # ament 资源文件
├── test/                          # ROS 标准测试
├── setup.py                       # Python 打包配置（4 个 entry_point）
├── package.xml                    # ROS 包清单（双系统条件依赖）
└── README.md                      # 英文文档
```

---

## 3. 话题接口

### Archer Y6 / Firefly Y6（标准机械臂 + 夹爪）

| 方向 | 话题 | 类型 | 说明 |
|------|------|------|------|
| 订阅 | `manip_ctrl` | `hex_ros_msgs/(msg/)HexRosRoboManipCtrlStamped` | 机械臂 + 夹爪控制指令 |
| 发布 | `manip_state` | `hex_ros_msgs/(msg/)HexRosRoboManipStateStamped` | 机械臂 + 夹爪状态反馈 |
| 发布 | `joint_states` | `sensor_msgs/(msg/)JointState` | 7 个关节（6 臂 + 1 夹爪） |
### Hello Y6（只读机械臂 + 摇杆手柄 + RGB LED）

| 方向 | 话题 | 类型 | 说明 |
|------|------|------|------|
| 发布 | `manip_state` | `hex_ros_msgs/(msg/)HexRosRoboManipStateStamped` | 机械臂状态反馈（夹爪为空） |
| 发布 | `joint_states` | `sensor_msgs/(msg/)JointState` | 6 个臂关节（无夹爪） |
| 发布 | `joy_state` | `hex_ros_msgs/(msg/)HexRosTeleopHandleStateStamped` | 摇杆手柄状态（axis_x/y、trigger、按钮 W/X/Y/Z） |
| 订阅 | `color_cmd` | `std_msgs/(msg/)ColorRGBA` | RGB LED 颜色指令（float 0-1 → int 0-255） |

> [消息类型描述](https://github.com/hexfellow/hex_ros_msgs#public-apis)

### MIT 模式使用警告
- 除非你知道什么是MIT模式，否则不要使用该模式
- 使用不当可能导致机械臂剧烈运动甚至损坏设备

> 确保在安全区域内运行，并随时准备急停

---

## 4. 控制模式

### Archer Y6 / Firefly Y6

**臂控制模式：** MIT 、JNT（位置）、EE（末端位姿）

**夹爪控制模式：** MIT 、JNT（位置）、TAU（力矩）

### Hello Y6

**无控制模式。** Hello Y6 是只读（state-only）设备，不支持下发控制指令。

---

## 5. 参数说明

### Archer Y6 / Firefly Y6

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `ctrl_rate` | 1000.0 | 主控制循环频率 [Hz] |
| `rate_state` | 500.0 | 状态发布频率（从 ctrl_rate 降采样）[Hz] |
| `robot_host` | 192.168.1.100 | 机器人控制器 IP 地址 |
| `robot_port` | 8439 | WebSocket 端口 |
| `robot_frame_id` | `base_link` | 状态消息中的坐标系 |
| `robot_grip_type` | `empty` | 夹爪类型：`gp80`、`gr100`或 `empty`（无夹爪） |
| `state_buffer_size` | 200 | 驱动状态缓冲区大小 |
| `sens_ts` | `true` | 是否使用传感器硬件时间戳 |

### Hello Y6

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `ctrl_rate` | 500.0 | 主控制循环频率 [Hz] |
| `rate_state` | 100.0 | 状态发布频率（从 ctrl_rate 降采样）[Hz] |
| `robot_host` | 192.168.1.100 | 机器人控制器 IP 地址 |
| `robot_port` | 8439 | WebSocket 端口 |
| `robot_frame_id` | `base_link` | 状态消息中的坐标系 |
| `state_buffer_size` | 200 | 驱动状态缓冲区大小 |
| `sens_ts` | `true` | 是否使用传感器硬件时间戳 |

> Hello Y6 无 `robot_grip_type` 参数（不涉及电机夹爪控制）。


> 默认提供ros时间；如果您需要硬件时间戳，可以通过`chs_state.jnt.header.stamp`获取

---

## 6. 依赖关系

### Python 包

```shell
pip3 install 'hex-util-msg>=0.1.0a0'
pip3 install 'hex-util-ros>=0.0.1a0'
pip3 install 'hex-util-runtime>=0.0.0,<0.1.0'
pip3 install 'hex-driver-robot>=0.0.1'
pip3 install 'hex_util_msg>=0.1.0'
```

### ROS 包

```shell
git clone https://github.com/hexfellow/hex_ros_msgs.git
git clone https://github.com/hexfellow/hex_ros_robot_arm.git
```

---

## 7. 快速使用

### 1. 构建工作空间

```shell
mkdir -p hex_ws/src
cd hex_ws/src
```

### 2. 克隆包

```shell
git clone https://github.com/hexfellow/hex_ros_msgs.git
git clone https://github.com/hexfellow/hex_ros_robot_arm.git
```

### 3. 编译包

**ROS 1：**

```shell
source /opt/ros/noetic/setup.bash
cd hex_ws
catkin_make
source devel/setup.bash --extend
```

**ROS 2：**

```shell
source /opt/ros/humble/setup.bash
cd hex_ws
colcon build
source install/setup.bash --extend
```

### 4. 使用包

```shell
# ROS 2 — Archer Y6
ros2 launch hex_ros_robot_arm archer.launch.py \
    robot_host:=192.168.1.100 robot_port:=8439 robot_grip_type:=empty

# ROS 2 — Firefly Y6
ros2 launch hex_ros_robot_arm firefly.launch.py \
    robot_host:=192.168.1.100 robot_port:=8439 robot_grip_type:=empty

# ROS 2 — Hello Y6（只读）
ros2 launch hex_ros_robot_arm hello.launch.py \
    robot_host:=192.168.1.100 robot_port:=8439

# ROS 2 — Archer Y6（带测试节点）
ros2 launch hex_ros_robot_arm archer.launch.py \
    robot_host:=192.168.1.100 robot_port:=8439 robot_grip_type:=empty test:=true
```

> 将 `robot_host` 和 `robot_port` 替换为实际机器人控制器的 IP 和端口。
> 我们提供了test_ctrl节点作为示例，您可以参考该节点实现控制。