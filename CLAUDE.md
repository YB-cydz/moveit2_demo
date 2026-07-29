# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

天轶2.0（Tianyi 2.0）双足双臂人形机器人的 ROS 2 + MoveIt 2 运动规划工作空间。包含两个包：

| 包 | 类型 | 说明 |
|---|---|---|
| `tianyi2_urdf` | 自研 | 机器人 URDF 模型、STL 网格（原始 + 凸包）、RViz 可视化 launch |
| `demo` | MoveIt 生成 | MoveIt Setup Assistant 生成的配置包（SRDF/运动学/控制器） |

MoveIt 2 通过 apt 安装（`ros-humble-moveit*`），无需源码编译。

## 构建与运行

```bash
cd /home/storm/moveit2_demo_ws
colcon build
source install/setup.bash
```

### 启动命令

```bash
# URDF 可视化（无 MoveIt）
ros2 launch tianyi2_urdf display.launch.py              # 无手部
ros2 launch tianyi2_urdf display_with_hands.launch.py    # 带手部
ros2 launch tianyi2_urdf display_convex.launch.py        # 凸包碰撞模型

# MoveIt 2 完整 Demo
ros2 launch demo demo.launch.py

# 单独启动各组件
ros2 launch demo move_group.launch.py
ros2 launch demo moveit_rviz.launch.py
ros2 launch demo rsp.launch.py
ros2 launch demo spawn_controllers.launch.py
```

## 架构

### URDF 加载链

```
demo.launch.py
  → MoveItConfigsBuilder("tianyi2_complete", package_name="demo")
    → tianyi2_complete.urdf.xacro    (demo/config/)
      ├── tianyi2_urdf_with_hands.urdf  (tianyi2_urdf/urdf/)  ← 实际机器人模型
      └── tianyi2_complete.ros2_control.xacro  (demo/config/) ← FakeSystem 仿真控制器
```

`MoveItConfigsBuilder` 是 `moveit_configs_utils` 提供的工具类，自动加载同目录下的 SRDF、kinematics.yaml、joint_limits.yaml 等配置文件。`demo.launch.py` 使用 `generate_demo_launch()` 一键生成完整 demo 启动。

### 当前配置的规划组

仅配置了**左臂** `hand_L_Arn_Group`（6 自由度），定义为从躯干到左手腕的 kinematic chain：

```
shoulder_pitch_l_link → shoulder_roll_l_link → shoulder_yaw_l_link
  → elbow_pitch_l_link → elbow_yaw_l_link → wrist_pitch_l_link
  → wrist_roll_l_link → L_base_link
```

包含关节：

| 关节 | 类型 |
|---|---|
| `shoulder_roll_l_joint` | 肩部侧摆 |
| `shoulder_yaw_l_joint` | 肩部偏航 |
| `elbow_pitch_l_joint` | 肘部俯仰 |
| `elbow_yaw_l_joint` | 肘部偏航 |
| `wrist_pitch_l_joint` | 腕部俯仰 |
| `wrist_roll_l_joint` | 腕部旋转 |

> 右臂（`*_r_joint` 系列）、双手手指关节、腿部、腰部、头部在 URDF 中存在，但**尚未在 SRDF 中配置为规划组**。若需添加，编辑 `src/demo/config/tianyi2_complete.srdf` 并同步更新 `kinematics.yaml` 和 `ros2_controllers.yaml`。

### 运动学求解

使用 KDL 插件，配置在 [kinematics.yaml](src/demo/config/kinematics.yaml)：
- `kinematics_solver`: `kdl_kinematics_plugin/KDLKinematicsPlugin`
- `kinematics_solver_search_resolution`: 0.005
- `kinematics_solver_timeout`: 0.005

### 控制器

仿真模式使用 `mock_components/GenericSystem`（FakeSystem），控制器为 `joint_trajectory_controller/JointTrajectoryController`。

### 网格

- `tianyi2_urdf/meshes/` — 原始 STL，用于**可视化**
- `tianyi2_urdf/meshes_convex/` — 凸包 STL，用于**碰撞检测**

---

## Python 运动控制

控制节点通过 ROS 2 action/service 与 `move_group` 节点通信。核心流程：

```
你的 Python 节点
  → set_pose_target / set_joint_value_target（move_group service）
  → plan() → JointTrajectory
  → execute() → FollowJointTrajectory action → ros2_control → 硬件
```

### 人形机器人参数（编写控制节点时使用）

```python
# 左臂 joint_names（来自 SRDF chain 定义）
JOINT_NAMES = [
    "shoulder_roll_l_joint", "shoulder_yaw_l_joint",
    "elbow_pitch_l_joint",   "elbow_yaw_l_joint",
    "wrist_pitch_l_joint",   "wrist_roll_l_joint",
]

# 关键 frame
BASE_LINK = "base"                    # 或 "shoulder_pitch_l_link"
END_EFFECTOR = "L_base_link"          # 左臂末端
GROUP_NAME = "hand_L_Arn_Group"       # SRDF 中的规划组名
HOME_POSE = "home"                    # SRDF 中预设的 group_state
```

### 创建新的控制包

```bash
ros2 pkg create tianyi2_control \
  --build-type ament_python \
  --dependencies rclpy std_msgs geometry_msgs trajectory_msgs
```

新节点入口点注册在 `setup.py` 的 `console_scripts`：
```python
entry_points={
    "console_scripts": [
        "pose_goal_node = tianyi2_control.pose_goal_node:main",
    ],
},
```

---

## 配置文件速查

| 文件 | 职责 |
|---|---|
| [tianyi2_complete.srdf](src/demo/config/tianyi2_complete.srdf) | 规划组定义、碰撞禁用矩阵、预设位姿（`home`） |
| [kinematics.yaml](src/demo/config/kinematics.yaml) | 运动学求解器配置（KDL） |
| [joint_limits.yaml](src/demo/config/joint_limits.yaml) | 关节速度和加速度限制 |
| [ros2_controllers.yaml](src/demo/config/ros2_controllers.yaml) | ros2_control 端控制器 |
| [moveit_controllers.yaml](src/demo/config/moveit_controllers.yaml) | MoveIt 端控制器映射 |
| [initial_positions.yaml](src/demo/config/initial_positions.yaml) | 仿真初始关节角度 |
| [pilz_cartesian_limits.yaml](src/demo/config/pilz_cartesian_limits.yaml) | Pilz 笛卡尔规划器限制 |

## 常见配置修改

- **添加规划组**：编辑 `tianyi2_complete.srdf`（`<group>` + `<group_state>`），同步更新 `kinematics.yaml` 和 `ros2_controllers.yaml`
- **更新碰撞矩阵**：编辑 `tianyi2_complete.srdf` 中的 `<disable_collisions>`
- **切换真实硬件**：修改 `tianyi2_complete.ros2_control.xacro` 的 `<plugin>` 和 `ros2_controllers.yaml`
- **重新生成 MoveIt 配置**：`ros2 launch demo setup_assistant.launch.py`

## 参考文档

- [docs/moveit2_guide.md](docs/moveit2_guide.md) — MoveIt 2 从配置到真机控制的全流程指南
