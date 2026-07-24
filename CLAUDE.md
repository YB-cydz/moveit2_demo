# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

天轶2.0（Tianyi 2.0）双足双臂人形机器人的 ROS 2 + MoveIt 2 运动规划工作空间。包含三个包：

| 包 | 类型 | 说明 |
|---|---|---|
| `tianyi2_urdf` | 自研 | 机器人 URDF 模型、STL 网格（原始 + 凸包）、RViz 可视化 launch |
| `demo` | MoveIt 生成 | MoveIt Setup Assistant 生成的配置包（SRDF/运动学/控制器） |
| `pymoveit2` | 第三方 | Python 版 MoveIt 2 接口库（AndrejOrsula/pymoveit2） |

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
- `tianyi2_urdf/meshes_convex/` — 由 `prime_to_convex.py` 生成的凸包 STL，用于**碰撞检测**

---

## pymoveit2 API（Python 开发核心）

所有 Python 运动控制代码应使用 `pymoveit2.MoveIt2` 类。

### 两种使用模式

**模式 1：分离 Plan + Execute**（默认，`use_move_group_action=False`）
```python
trajectory = moveit2.plan(position=[0.3, 0.0, 0.5], quat_xyzw=[0.0, 0.0, 0.0, 1.0])
moveit2.execute(trajectory)
moveit2.wait_until_executed()
```

**模式 2：合并 Plan + Execute**（`use_move_group_action=True`）
```python
moveit2.move_to_pose(position=[0.3, 0.0, 0.5], quat_xyzw=[0.0, 0.0, 0.0, 1.0])
moveit2.wait_until_executed()
```

### 关键 API 方法

| 方法 | 说明 |
|---|---|
| `plan(position=, quat_xyzw=)` | 运动学路径规划（位姿目标） |
| `plan(joint_positions=)` | 运动学路径规划（关节目标） |
| `plan(cartesian=True)` | 笛卡尔空间路径规划 |
| `plan_async(...)` | 异步规划，返回 Future |
| `execute(trajectory)` | 执行已规划的轨迹 |
| `move_to_pose(...)` | 规划 + 执行（位姿目标） |
| `move_to_configuration(...)` | 规划 + 执行（关节目标） |
| `compute_fk(joint_state=)` | 正运动学 |
| `compute_ik(position=, quat_xyzw=)` | 逆运动学 |
| `add_collision_box/sphere/cylinder/cone/mesh(...)` | 添加碰撞对象 |
| `attach_collision_object(id, link_name)` | 附加碰撞对象到机器人 |
| `remove_collision_object(id)` | 移除碰撞对象 |
| `query_state()` | 查询当前执行状态（IDLE/REQUESTING/EXECUTING） |

### 关键属性

- `max_velocity` / `max_acceleration` — 速度/加速度缩放因子（0.0~1.0）
- `planner_id` — 规划器选择（如 `"RRTConnectkConfigDefault"`）
- `num_planning_attempts` / `allowed_planning_time` — 规划参数
- `joint_state` — 当前关节状态（只读）

### 初始化参数

```python
MoveIt2(
    node=node,
    joint_names=["shoulder_roll_l_joint", "shoulder_yaw_l_joint",
                 "elbow_pitch_l_joint", "elbow_yaw_l_joint",
                 "wrist_pitch_l_joint", "wrist_roll_l_joint"],
    base_link_name="base",           # 或 "shoulder_pitch_l_link"
    end_effector_name="L_base_link", # 或特定工具 link
    group_name="hand_L_Arn_Group",
    callback_group=callback_group,
)
```

> **注意**：pymoveit2 的 `robots/` 模块目前只有 Panda 机器人的预设。天轶2.0 **没有自己的 robot preset**，需在代码中手动指定 `joint_names`、`base_link_name`、`end_effector_name`。这些信息可从 SRDF 的 `<chain>` 定义和 URDF 中提取。

### 异步执行模式

pymoveit2 支持异步操作，参考 [ex_pose_goal.py](src/pymoveit2/examples/ex_pose_goal.py)：
```python
# 需要在后台线程中 spin 节点
executor = rclpy.executors.MultiThreadedExecutor(2)
executor.add_node(node)
executor_thread = Thread(target=executor.spin, daemon=True)
executor_thread.start()

# 异步执行
moveit2.move_to_pose(position=..., quat_xyzw=..., cartesian=False)
future = moveit2.get_execution_future()
while not future.done():
    ...
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

---

## 常见配置修改

- **添加规划组**：编辑 `tianyi2_complete.srdf`（`<group>` + `<group_state>`），同步更新 `kinematics.yaml` 和 `ros2_controllers.yaml`
- **更新碰撞矩阵**：编辑 `tianyi2_complete.srdf` 中的 `<disable_collisions>`
- **切换真实硬件**：修改 `tianyi2_complete.ros2_control.xacro` 的 `<plugin>` 和 `ros2_controllers.yaml`
- **重新生成 MoveIt 配置**：`ros2 launch demo setup_assistant.launch.py`

## 辅助脚本（`tianyi2_urdf/script/`）

| 脚本 | 功能 |
|---|---|
| `check_link_and_update.py` | 将 CSV 关键参数表中的 link 名称与 URDF 交叉校核 |
| `check_joint_and_update.py` | 将 CSV 关键参数表中的 joint 参数与 URDF 交叉校核 |
| `make_mjcf_torq.py` | 将 URDF 转换为 MuJoCo MJCF 格式 |
| `prime_to_convex.py` | 从原始 STL 生成凸包 STL（依赖 trimesh），用于碰撞检测网格更新 |
