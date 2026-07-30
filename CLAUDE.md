# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

> **重要规则**：执行任何命令行操作（`ros2`、`colcon`、`git`、`apt` 等）前，必须先说明意图并得到用户明确许可。

## 项目概述

天轶2.0（Tianyi 2.0）双足双臂人形机器人的 ROS 2 + MoveIt 2 运动规划工作空间。包含三个包：

| 包 | 类型 | 说明 |
|---|---|---|
| `tianyi2_urdf` | 自研 | 机器人 URDF 模型、STL 网格（原始 + 凸包）、RViz 可视化 launch |
| `demo` | MoveIt 生成 | MoveIt Setup Assistant 生成的配置包（SRDF/运动学/控制器） |
| `tianyi2_control` | 自研 | Python 控制节点（位姿目标 + 双臂支持），直调 move_group service/action |

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

配置了**左臂** `L_Arm_Group` 和**右臂** `R_Arm_group`，各 6 自由度：

**左臂** `L_Arm_Group`：
```
shoulder_pitch_l_link → shoulder_roll_l_link → shoulder_yaw_l_link
  → elbow_pitch_l_link → elbow_yaw_l_link → wrist_pitch_l_link
  → wrist_roll_l_link → L_base_link
```

**右臂** `R_Arm_group`：
```
shoulder_pitch_r_link → shoulder_roll_r_link → shoulder_yaw_r_link
  → elbow_pitch_r_link → elbow_yaw_r_link → wrist_pitch_r_link
  → wrist_roll_r_link → R_base_link
```

包含关节：

| 左臂关节 | 右臂关节 | 类型 |
|---|---|---|
| `shoulder_roll_l_joint` | `shoulder_roll_r_joint` | 肩部侧摆 |
| `shoulder_yaw_l_joint` | `shoulder_yaw_r_joint` | 肩部偏航 |
| `elbow_pitch_l_joint` | `elbow_pitch_r_joint` | 肘部俯仰 |
| `elbow_yaw_l_joint` | `elbow_yaw_r_joint` | 肘部偏航 |
| `wrist_pitch_l_joint` | `wrist_pitch_r_joint` | 腕部俯仰 |
| `wrist_roll_l_joint` | `wrist_roll_r_joint` | 腕部旋转 |

### 运动学求解

使用 KDL 插件，左臂和右臂均配置在 [kinematics.yaml](src/demo/config/kinematics.yaml)。
- `kinematics_solver`: `kdl_kinematics_plugin/KDLKinematicsPlugin`
- `kinematics_solver_search_resolution`: 0.005
- `kinematics_solver_timeout`: 0.005

### 控制器

仿真模式使用 `mock_components/GenericSystem`（FakeSystem），控制关节通过 `joint_trajectory_controller/JointTrajectoryController`；仅上报状态的被动关节（腰部、腿部、shoulder_pitch）只注册 `state_interface` 以补全 TF 链。

### TF 链

从 `base` 到 `L_base_link` / `R_base_link` 的完整链需要 `/joint_states` 包含 18 个关节（12 个手臂控制关节 + 6 个被动关节）。若 TF 断开，检查 `ros2_control.xacro` 和 `initial_positions.yaml` 是否包含了链上所有关节。详见 [docs/tf_configuration.md](docs/tf_configuration.md)。

### 网格

- `tianyi2_urdf/meshes/` — 原始 STL，用于**可视化**
- `tianyi2_urdf/meshes_convex/` — 凸包 STL，用于**碰撞检测**

---

## Python 运动控制

控制节点 (`tianyi2_control`) 通过 ROS 2 action/service 与 `move_group` 节点通信。

```
/left_arm/target_pose  → 左臂绝对位姿
/left_arm/delta_pose   → 左臂相对变换（平移base系 + 旋转末端系）
/right_arm/target_pose → 右臂绝对位姿
/right_arm/delta_pose  → 右臂相对变换
```

### 人形机器人参数（编写控制节点时使用）

```python
BASE_LINK = "base"

# 左臂
L_GROUP = "L_Arm_Group"
L_EFFECTOR = "L_base_link"
L_JOINTS = ["shoulder_roll_l_joint", "shoulder_yaw_l_joint",
            "elbow_pitch_l_joint", "elbow_yaw_l_joint",
            "wrist_pitch_l_joint", "wrist_roll_l_joint"]

# 右臂
R_GROUP = "R_Arm_group"
R_EFFECTOR = "R_base_link"
R_JOINTS = ["shoulder_roll_r_joint", "shoulder_yaw_r_joint",
            "elbow_pitch_r_joint", "elbow_yaw_r_joint",
            "wrist_pitch_r_joint", "wrist_roll_r_joint"]
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
- [docs/create_control_pkg.md](docs/create_control_pkg.md) — 创建控制包 + 节点完整代码
- [docs/tf_configuration.md](docs/tf_configuration.md) — TF 配置与故障排查
- [docs/testing_guide.md](docs/testing_guide.md) — 测试步骤与常用命令
