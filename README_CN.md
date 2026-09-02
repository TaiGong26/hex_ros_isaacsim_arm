# hex_ros_isaacsim_arm
**中文** | [English](README.md)

## 目录

- [1. 包的简介](#1-包的简介)
- [2. 包架构](#2-包架构)
- [3. 话题接口](#3-话题接口)
- [4. 控制模式](#4-控制模式)
- [5. 参数说明](#5-参数说明)
- [6. 依赖关系](#6-依赖关系)
- [7. Isaacsim Action Graph](#7-isaacsim-action-graph)
- [8. 快速使用](#8-快速使用)
- [9. 常见问题](#9-常见问题)

---

## 1. 包的简介

`hex_ros_isaacsim_arm` 是 Isaac Sim Archer Y6 机械臂的 ROS 2 Bridge 转发包。

本包负责：

- 订阅上游发布的 `manip_ctrl` 机械臂和夹爪控制指令；
- 订阅 Isaac Sim Bridge 发布的关节状态；
- 将支持的控制指令转发为 Isaac Sim Bridge 使用的 `sensor_msgs/msg/JointState` 命令；
- 通过参数指定状态话题和命令话题。

本包包含 Archer Y6 节点，并支持以下夹爪配置：

- `empty`：不使用夹爪；
- `gp80`：使用 GP80 夹爪；
- `gr100`：使用 GR100 夹爪。

本包不连接真实机械臂控制器，也不负责发布真实机械臂驱动包中的 `manip_state`。

---

## 2. 包架构

```text
hex_ros_isaacsim_arm/
├── config/ros2/
│   └── archer_params.yaml
├── launch/ros2/
│   └── isaacsim_archer_y6.launch.py
├── hex_ros_isaacsim_arm/
│   ├── isaacsim_archer_y6.py
│   └── utility/
├── package.xml
├── setup.py
└── README_CN.md
```

### 节点入口

| 节点 | 可执行文件 | 作用 |
|---|---|---|
| Archer Y6 | `isaacsim_archer_y6` | 转发 Archer 机械臂和夹爪控制指令 |

---

## 3. 话题接口

| 方向 | 话题/参数 | 默认话题 | 类型 | 说明 |
|---|---|---|---|---|
| 订阅 | `manip_ctrl` | `manip_ctrl` | `hex_ros_msgs/msg/HexRosRoboManipCtrlStamped` | 机械臂和夹爪控制输入 |
| 订阅 | `joint_state_topic` | `/joint_states` | `sensor_msgs/msg/JointState` | Isaac Sim Bridge 关节状态 |
| 发布 | `joint_command_topic` | `/joint_command` | `sensor_msgs/msg/JointState` | Isaac Sim Bridge 关节命令 |

`joint_state_topic` 和 `joint_command_topic` 可以在 `config/ros2/archer_params.yaml` 中自定义。修改后，Isaac Sim Bridge 的状态发布和命令订阅话题必须与参数保持一致。

### JointState 字段

对于 `/joint_command`：

- `name`：关节名称及其数组顺序；
- `position`：关节位置目标；
- `effort`：关节力矩或前馈力矩字段。

---

## 4. 控制模式

### Arm

| 模式 | 状态 | 说明 |
|---|---|---|
| `JNT` | 支持 | 关节位置控制 |
| `EE` | 支持 | 末端位姿控制 |
| `MIT` | 不支持 | 输出 warning，不发布该控制命令 |

### Gripper

| 模式 | 状态 | 说明 |
|---|---|---|
| `NONE` | 支持 | 空模式，清除当前保留控制指令 |
| `JNT` | 支持 | 关节位置控制 |
| `TAU` | 支持 | 力矩控制，最大绝对力矩为 `40 N·m` |
| `MIT` | 不支持 | 输出 warning，不发布该控制命令 |

`NONE` 是空模式，不代表额外发送一个位置或力矩控制目标。Arm 和 gripper 收到不支持的 MIT 控制时，不会自动转换为其他模式。

---

## 5. 参数说明

参数文件：`config/ros2/archer_params.yaml`

| 参数 | 默认值 | 说明 |
|---|---|---|
| `ctrl_rate` | `1000.0` | 控制命令转发频率 [Hz] |
| `use_sim_time` | `true` | 是否使用 ROS 仿真时间 |
| `robot_grip_type` | `gr100` | 夹爪类型：`empty`、`gp80` 或 `gr100` |
| `urdf_path` | `""` | Archer URDF 路径，用于 EE 控制模型 |
| `joint_state_topic` | `/joint_states` | 可自定义的 Bridge 状态话题 |
| `joint_command_topic` | `/joint_command` | 可自定义的 Bridge 命令话题 |

启动文件会为 `urdf_path` 提供 `hex_ros_urdf_archer_y6` 包中的 `empty.urdf` 路径覆盖值。如果直接运行节点或覆盖该参数，需要提供有效的 URDF 路径才能使用 EE 控制。

> 如果 `use_sim_time` 为 `true`，需要在 Isaac Sim 中开启 `/clock` 话题。

---

## 6. 依赖关系

### Python 包

本包使用 ROS 2 Python 环境和以下 Python 依赖：

```shell
pip3 install 'hex-util-msg>=0.1.0a4'
pip3 install 'hex-util-ros>=0.0.1a6'
```


### ROS 包

创建 ROS 2 工作空间并获取消息包和 Archer URDF 包：

```shell
mkdir -p <your_ws>/src
cd <your_ws>/src
git clone https://github.com/hexfellow/hex_ros_msgs.git
git clone https://github.com/hexfellow/hex_ros_urdf_archer_y6.git
git clone https://github.com/hexfellow/hex_ros_isaacsim_arm.git
```

### Isaac Sim ROS 2 Bridge

Isaac Sim 侧需要启用 ROS 2 Bridge，并创建与本包参数一致的 `JointState` 状态发布和命令订阅接口。安装和启用方式请参考 Isaac Sim 官方文档：

- [Isaac Sim ROS 2 Installation](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_ros.html)

Isaac Sim 场景还需要对应的 USD 资源。请获取以下 USD 资源仓库，并按照仓库说明配置资产路径：

```shell
git clone https://github.com/hexfellow/hex_isaac_usd.git
```

---

## 7. Isaacsim Action Graph

![Action Graph](./img/80d5a0cb2d9ced03a2dc84bb76663a84.png)

---

## 8. 快速使用

### 1. 编译工作空间

```shell
mkdir -p <your_ws>/src
cd <your_ws>/src
git clone https://github.com/hexfellow/hex_ros_msgs.git
git clone https://github.com/hexfellow/hex_ros_urdf_archer_y6.git
git clone https://github.com/hexfellow/hex_ros_isaacsim_arm.git

cd <your_ws>
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash --extend
```

### 2. 启动节点

```shell
ros2 launch hex_ros_isaacsim_arm isaacsim_archer_y6.launch.py
```

### 3. 查看接口

```shell
ros2 node info /hex_ros_isaacsim_archer_y6
ros2 topic list -t
ros2 topic info /manip_ctrl -v
ros2 topic info /joint_states -v
ros2 topic info /joint_command -v
ros2 topic echo /joint_states --once
ros2 topic echo /joint_command --once
```

### 4. 发布 Arm EE 指令

以下示例将末端位置设置为 `{0.3, 0.0, 0.3}` m，姿态使用单位四元数：

```shell
ros2 topic pub --rate 10 /manip_ctrl \
  hex_ros_msgs/msg/HexRosRoboManipCtrlStamped \
  '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "arm_link"}, manip_ctrl: {arm_ctrl: {ctrl_mode: 3, jnt: {pos: [], vel: [], eff: [], kp: [], kd: [], lim_vel: [1.0], lim_acc: [50.0]}, pose: {position: {x: 0.3, y: 0.0, z: 0.3}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, grip_ctrl: {ctrl_mode: 0, jnt: {pos: [], vel: [], eff: [], kp: [], kd: [], lim_vel: [], lim_acc: []}}}}'
```

### 5. 发布 Arm JNT 指令

```shell
ros2 topic pub --rate 10 /manip_ctrl \
  hex_ros_msgs/msg/HexRosRoboManipCtrlStamped \
  '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "arm_link"}, manip_ctrl: {arm_ctrl: {ctrl_mode: 2, jnt: {pos: [0.0, -1.3, 2.8, 0.0, 0.0, 0.0], vel: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], eff: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], lim_vel: [1.0], lim_acc: [50.0]}}, grip_ctrl: {ctrl_mode: 0, jnt: {pos: [], vel: [], eff: [], kp: [], kd: [], lim_vel: [], lim_acc: []}}}}'
```

### 6. 发布 Gripper JNT 指令

以下示例适用于双关节 `gr100` 配置：

```shell
# Gripper position: {0.0, 0.0}
ros2 topic pub --rate 10 /manip_ctrl \
  hex_ros_msgs/msg/HexRosRoboManipCtrlStamped \
  '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "arm_link"}, manip_ctrl: {arm_ctrl: {ctrl_mode: 2, jnt: {pos: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}}, grip_ctrl: {ctrl_mode: 2, jnt: {pos: [0.0, 0.0], lim_vel: [0.5, 0.5]}}}}'

# Gripper position: {0.5, 0.5}
ros2 topic pub --rate 10 /manip_ctrl \
  hex_ros_msgs/msg/HexRosRoboManipCtrlStamped \
  '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "arm_link"}, manip_ctrl: {arm_ctrl: {ctrl_mode: 2, jnt: {pos: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}}, grip_ctrl: {ctrl_mode: 2, jnt: {pos: [0.5, 0.5], lim_vel: [0.5, 0.5]}}}}'
```

使用 `gp80` 或其他配置时，position 数组长度应与实际 gripper 关节数量一致。

### 7. 清除控制

向 `manip_ctrl` 发布 arm 和 gripper 均为 `NONE` 的空模式消息，以清除当前保留控制指令。具体嵌套字段应以当前 `hex_ros_msgs` 定义为准。

---

## 9. 常见问题

### Arm 或 Gripper MIT 被拒绝

这是当前设计：

```text
Arm:     JNT、EE
Gripper: NONE、JNT、TAU
```

MIT 会输出 warning，不会自动转换为其他控制模式，也不会发布该控制命令。

### EE 控制不可用

检查：

```shell
ros2 param get /hex_ros_isaacsim_archer_y6 urdf_path
```

确认 `urdf_path` 指向有效的 Archer URDF。没有有效 URDF 时，JNT arm 控制仍可使用，但 EE 控制不可用。

### Gripper TAU 模式力矩限制

Gripper TAU 输出的最大绝对力矩为：

```text
40 N·m
```

### 没有收到 `/joint_states`

检查：

```shell
ros2 topic info /joint_states -v
ros2 topic echo /joint_states --once
```

如果没有实际消息，节点无法确认 Bridge 关节接口，不能正常发布命令。请检查 Isaac Sim ROS 2 Bridge、ROS domain、RMW、DDS discovery 和 Docker 网络配置。

### 修改 Bridge 话题

在 `archer_params.yaml` 中修改：

```yaml
joint_state_topic: "/my_isaac_joint_states"
joint_command_topic: "/my_isaac_joint_command"
```

Isaac Sim Bridge 的状态发布和命令订阅也必须使用相同的话题名称。

### `NONE` 的含义

`NONE` 是空模式，用于清除当前保留控制指令。它不是一次新的 JNT、EE 或 TAU 控制。
