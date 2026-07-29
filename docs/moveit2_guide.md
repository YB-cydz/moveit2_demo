# 天轶2.0 MoveIt 2 完整指南

> 基于 MoveIt 2 官方文档 + 你的项目配置，从配置到写控制节点到对接真实硬件的全流程。

---

## 1. 架构概览：MoveIt 2 是怎么控制机器人的

```
┌─────────────────────────────────────────────────────────────────────┐
│                       你的 Python 节点                               │
│    使用 moveit_py API 发送目标（位姿/关节）                           │
└──────────────────────────┬──────────────────────────────────────────┘
                           │ ROS 2 service/action 调用
                           ▼
┌─────────────────────────────────────────────────────────────────────┐
│                      move_group 节点                                │
│  • 维护 Planning Scene（世界模型 + 机器人状态）                       │
│  • 调用 IK 求解器 → 关节角度                                         │
│  • 调用运动规划器 → 无障碍路径                                       │
│  • 调用 Trajectory Execution Manager → 发送给控制器                  │
└──────────────────────────┬──────────────────────────────────────────┘
                           │ FollowJointTrajectory action
                           ▼
┌─────────────────────────────────────────────────────────────────────┐
│                   ros2_control (controller_manager)                  │
│  • JointTrajectoryController 接收轨迹                                │
│  • JointStateBroadcaster 发布关节状态反馈                             │
└──────────────────────────┬──────────────────────────────────────────┘
                           │
                           ▼
┌──────────────────────────────────┐  ┌───────────────────────────────┐
│   仿真：mock_components           │  │   真机：你的 Hardware Interface  │
│   (FakeSystem)                   │  │   (通过 CAN/EtherCAT 发指令)    │
└──────────────────────────────────┘  └───────────────────────────────┘
```

**关键点：**
- MoveIt **不直接**控制电机。它输出 `JointTrajectory`（一串关节角度+时间戳）
- 控制器（`JointTrajectoryController`）接收轨迹后，逐点插值发给硬件
- `robot_state_publisher` + `joint_state_broadcaster` 发布 TF 和关节状态供 MoveIt 读取

---

## 2. 你的配置文件详解

你已经通过 MoveIt Setup Assistant 生成了 `demo` 包。下面逐一解释每个文件的作用：

### 2.1 [tianyi2_complete.srdf](../../src/demo/config/tianyi2_complete.srdf) — 语义描述文件

```xml
<!-- 规划组定义：左臂 6-DOF kinematic chain -->
<group name="hand_L_Arn_Group">
    <chain base_link="shoulder_pitch_l_link" tip_link="L_base_link"/>
</group>

<!-- 预设位姿 "home" -->
<group_state name="home" group="hand_L_Arn_Group">
    <joint name="shoulder_roll_l_joint" value="1.3611"/>
    <joint name="shoulder_yaw_l_joint" value="1.4589"/>
    <joint name="elbow_pitch_l_joint"  value="-0.9633"/>
    <joint name="elbow_yaw_l_joint"    value="0.7704"/>
    <joint name="wrist_pitch_l_joint"  value="-0.0058"/>
    <joint name="wrist_roll_l_joint"   value="-0.2483"/>
</group_state>

<!-- 碰撞禁用矩阵：800+ 对 link 永不检测碰撞 -->
<disable_collisions link1="..." link2="..." reason="Never|Adjacent|Default"/>
```

**作用：**
- `<group>` 定义了 MoveIt 能规划哪些关节。你的 SRDF **只有左臂**。
- `<group_state name="home">` 让你在代码中用 `set_goal_state(configuration_name="home")` 一键回到预设位姿
- `<disable_collisions>` 的 `reason="Never"` 表示这两部件永远不可能碰撞，跳过检测以加速规划

**添加右臂：** 在 `<robot>` 内增加：
```xml
<group name="hand_R_Arn_Group">
    <chain base_link="shoulder_pitch_r_link" tip_link="R_base_link"/>
</group>
<group_state name="home" group="hand_R_Arn_Group">
    <!-- 右侧镜像关节值 -->
</group_state>
```

### 2.2 [kinematics.yaml](../../src/demo/config/kinematics.yaml) — 运动学求解器

```yaml
hand_L_Arn_Group:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005   # IK 搜索步长 (rad)
  kinematics_solver_timeout: 0.005              # IK 超时 (秒)
```

**作用：** 指定用 KDL（Kinematica and Dynamics Library）做逆运动学求解。KDL 是数值迭代法，速度快但可能掉进局部最优。

### 2.3 [joint_limits.yaml](../../src/demo/config/joint_limits.yaml) — 关节限位

```yaml
default_velocity_scaling_factor: 0.1    # 默认速度 10%（安全值）
default_acceleration_scaling_factor: 0.1

joint_limits:
  shoulder_roll_l_joint:
    has_velocity_limits: true
    max_velocity: 3.9793                # rad/s
    has_acceleration_limits: false
```

**作用：**
- 覆盖 URDF 中的速度/加速度限制
- `default_velocity_scaling_factor: 0.1` 意味着默认以 10% 速度运行，代码中可通过 `max_velocity` 属性调整
- 如果 URDF 已有准确限位，可以调高到 1.0

### 2.4 [moveit_controllers.yaml](../../src/demo/config/moveit_controllers.yaml) — MoveIt → 控制器映射

```yaml
moveit_simple_controller_manager:
  controller_names:
    - hand_L_Arn_Group_controller

  hand_L_Arn_Group_controller:
    type: FollowJointTrajectory          # ← 接口类型
    joints:
      - shoulder_roll_l_joint
      - shoulder_yaw_l_joint
      - elbow_pitch_l_joint
      - elbow_yaw_l_joint
      - wrist_pitch_l_joint
      - wrist_roll_l_joint
    action_ns: follow_joint_trajectory   # ← action namespace
    default: true
```

**作用：** MoveIt 通过 `/<controller_name>/follow_joint_trajectory` action 将轨迹发给控制器。`controller_name` **必须**与 [ros2_controllers.yaml](../../src/demo/config/ros2_controllers.yaml) 中的一致。

### 2.5 [ros2_controllers.yaml](../../src/demo/config/ros2_controllers.yaml) — ros2_control 端控制器

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz，控制循环频率
    hand_L_Arn_Group_controller:
      type: joint_trajectory_controller/JointTrajectoryController
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

hand_L_Arn_Group_controller:
  ros__parameters:
    joints: [shoulder_roll_l_joint, shoulder_yaw_l_joint, ...]
    command_interfaces:
      - position                        # ← 位置控制
    state_interfaces:
      - position
      - velocity
```

**作用：**
- `controller_manager` 加载两个控制器：
  - `JointTrajectoryController` — 执行轨迹（接收 MoveIt 发来的关节序列）
  - `JointStateBroadcaster` — 发布 `/joint_states`（MoveIt 需要它来知道机器人当前状态）
- `command_interfaces: [position]` 表示位置控制模式（也可以是 velocity/effort）
- `update_rate: 100` 是控制循环 100Hz

### 2.6 [initial_positions.yaml](../../src/demo/config/initial_positions.yaml) — 初始关节角

```yaml
initial_positions:
  elbow_pitch_l_joint: 0
  elbow_yaw_l_joint: 0
  ...
```

仿真启动时所有关节归零。

### 2.7 [demo.launch.py](../../src/demo/launch/demo.launch.py) — 一键启动

```python
moveit_config = MoveItConfigsBuilder("tianyi2_complete", package_name="demo").to_moveit_configs()
return generate_demo_launch(moveit_config)
```

`generate_demo_launch()` 内部启动：`move_group` + `rviz` + `robot_state_publisher` + `ros2_control` + `joint_state_broadcaster`。

---

## 3. 使用 moveit_py 编写控制节点（官方 API）

### 3.1 最小示例：位姿目标

```python
#!/usr/bin/env python3
"""MoveIt 2 官方 Python API：位姿目标控制"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy, MultiPipelinePlanRequestParameters


def plan_and_execute(robot, planning_component,
                     single_plan_params=None, multi_plan_params=None):
    """规划并执行。planning 失败返回 None。"""
    if multi_plan_params is not None:
        plan_result = planning_component.plan(
            multi_plan_parameters=multi_plan_params)
    elif single_plan_params is not None:
        plan_result = planning_component.plan(
            single_plan_parameters=single_plan_params)
    else:
        plan_result = planning_component.plan()  # 使用默认参数

    if plan_result:
        robot_trajectory = plan_result.trajectory
        robot.execute(robot_trajectory, controllers=[])
        return robot_trajectory
    else:
        print("Planning failed")
        return None


def main():
    rclpy.init()
    logger = rclpy.logging.get_logger("moveit_py.pose_goal")

    # ---- 创建 MoveItPy 实例 ------------------------------------------------
    # MoveItPy 自动从参数服务器加载 SRDF/kinematics/URDF 等配置
    # 注意：必须先 launch demo.launch.py 或 move_group.launch.py
    tianyi = MoveItPy(node_name="moveit_py")

    # 获取规划组件（名称对应 SRDF 中的 <group name="...">）
    left_arm = tianyi.get_planning_component("hand_L_Arn_Group")
    logger.info("MoveItPy 实例已创建")

    # ---- 设置起始状态为当前状态 -------------------------------------------
    left_arm.set_start_state_to_current_state()

    # ---- 方法 1：笛卡尔位姿目标（set_goal_state with PoseStamped）-------
    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "base"          # 参考坐标系（你的 URDF 中的 base link）
    pose_goal.pose.orientation.w = 1.0           # 四元数 wxyz 格式，w=1 表示无旋转
    pose_goal.pose.orientation.x = 0.0
    pose_goal.pose.orientation.y = 0.0
    pose_goal.pose.orientation.z = 0.0
    pose_goal.pose.position.x = 0.3
    pose_goal.pose.position.y = 0.0
    pose_goal.pose.position.z = 0.5
    left_arm.set_goal_state(
        pose_stamped_msg=pose_goal,
        pose_link="L_base_link"                 # 末端执行器 link
    )

    # ---- 方法 2：预设位姿目标 -------------------------------------------
    # left_arm.set_goal_state(configuration_name="home")

    # ---- 规划并执行 -------------------------------------------------------
    plan_and_execute(tianyi, left_arm)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

### 3.2 关键 API 详解

**创建 MoveItPy 实例：**
```python
tianyi = MoveItPy(node_name="moveit_py")
left_arm = tianyi.get_planning_component("hand_L_Arn_Group")
```
- `MoveItPy` 连接 `move_group` 节点的所有 service/action
- `get_planning_component(name)` 的 `name` 对应 SRDF 中 `<group name="...">`

**设置目标（三种方式）：**
```python
# 方式 1：位姿目标 → MoveIt 自动调 IK
left_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="L_base_link")

# 方式 2：关节目标 → 跳过 IK，直接规划
left_arm.set_goal_state(robot_state=joint_state)

# 方式 3：SRDF 中预定义的 group_state
left_arm.set_goal_state(configuration_name="home")
```

**设置起始状态：**
```python
left_arm.set_start_state_to_current_state()   # 从 /joint_states 读取
# left_arm.set_start_state("home")            # 从预设位姿开始（可能与当前状态冲突）
```

### 3.3 笛卡尔空间路径规划（直线走）

笛卡尔规划 = 末端执行器沿直线移动。底层通过插值中间点 + IK 求解实现。

```python
# ---- 定义一系列路径点（waypoints）-------------------------------------
# MoveIt 会在相邻 waypoint 之间插值生成直线轨迹
waypoints = []

# 起点：当前位置
start_pose = PoseStamped()
start_pose.header.frame_id = "base"
start_pose.pose = left_arm.get_current_state().getGlobalLinkTransform("L_base_link")
waypoints.append(start_pose.pose)

# 中间点：向左移动
wp = start_pose.pose
wp.position.y += 0.1
waypoints.append(wp)

# 终点：向下移动
wp.position.z -= 0.15
waypoints.append(wp)

# ---- 规划笛卡尔路径 ---------------------------------------------------
# C++ API: computeCartesianPath(waypoints, eef_step, trajectory)
# Python API 通过 moveit_py 的 plan() 配合 single_plan_parameters
# 当前 moveit_py 没有直接暴露 compute_cartesian_path，
# 笛卡尔规划通过设置 pilz_lin planner 实现

# 方式：使用 Pilz LIN 规划器（笛卡尔直线插值）
params = SinglePlanRequestParameters(
    planner_id="PilzLin",
    planning_pipeline="pilz_industrial_motion_planner",
    # ... 其他参数
)
plan_and_execute(tianyi, left_arm, single_plan_params=params)
```

### 3.4 获取并检查规划的路径

`plan()` 返回的 `plan_result` 包含完整轨迹：

```python
plan_result = left_arm.plan()

if plan_result:
    # ---- 取出轨迹对象 --------------------------------------------------
    trajectory = plan_result.trajectory  # RobotTrajectory 类型

    # ---- 转换为 JointTrajectory 消息（可直接查看每个路径点）------------
    traj_msg = trajectory.get_robot_trajectory_msg()
    jt = traj_msg.joint_trajectory  # trajectory_msgs/JointTrajectory

    print(f"关节名称: {jt.joint_names}")
    print(f"路径点数量: {len(jt.points)}")

    # ---- 遍历每个路径点 ------------------------------------------------
    for i, point in enumerate(jt.points):
        # point.positions: 各关节角度 (rad)
        # point.velocities: 各关节速度 (rad/s)
        # point.time_from_start: 从轨迹起点至此点的时间 (秒)
        print(f"  点 {i}: pos={[f'{p:.4f}' for p in point.positions]}, "
              f"t={point.time_from_start.sec + point.time_from_start.nanosec * 1e-9:.3f}s")

    # ---- 执行轨迹 -------------------------------------------------------
    tianyi.execute(trajectory, controllers=[])
else:
    print("规划失败")

# ---- 独立的 IK 查询 --------------------------------------------------------
# 需要访问 RobotModel 和 RobotState（较低层 API）
```

---

## 4. 多规划器并行：生成多条路径并选择最佳

MoveIt 支持**同时跑多个规划器**，然后选择最好的一条。

```python
from moveit.planning import MultiPipelinePlanRequestParameters

# ---- 设置目标 -----------------------------------------------------------
left_arm.set_start_state_to_current_state()
left_arm.set_goal_state(configuration_name="home")

# ---- 定义多个规划管道并行运行 -----------------------------------------
multi_params = MultiPipelinePlanRequestParameters(
    tianyi,
    ["ompl_rrtc",              # RRTConnect — 快速
     "pilz_lin",               # Pilz 直线插值
     "chomp",                  # CHOMP — 优化轨迹质量
     "ompl_rrt_star"]          # RRT* — 渐进最优
)

# ---- 规划（并行）并执行 ------------------------------------------------
# plan() 默认会选择第一个成功的方案
# 也可以自定义 solution_selection_function 来按规则选择
plan_result = left_arm.plan(multi_plan_parameters=multi_params)
if plan_result:
    print(f"规划成功，轨迹有 {len(plan_result.trajectory.get_robot_trajectory_msg().joint_trajectory.points)} 个点")
    tianyi.execute(plan_result.trajectory, controllers=[])
```

**解决方案选择函数（自定义选择逻辑）：**
```python
# plan() 支持传入 solution_selection_function 回调
# 例如选择最短路径、最平滑、最远离关节限位等

# 伪代码——Python binding 中的具体签名取决于你的 moveit_py 版本
# 通常可以遍历多个 plan_result 比较 path_length 或 clearance
```

**可用的规划器：**

| 规划器 ID | 说明 |
|---|---|
| `ompl_rrtc` | RRT-Connect，速度快，适合常见场景 |
| `ompl_rrt_star` | RRT*，渐进最优但慢 |
| `pilz_lin` | 笛卡尔直线（要求直线路径） |
| `pilz_ptp` | 点到点（关节空间） |
| `chomp` | 轨迹优化，平滑但可能慢 |
| `stomp` | 随机轨迹优化 |

---

## 5. 轨迹如何控制关节（从 MoveIt 到实体机器人）

### 5.1 完整数据流

```
你的代码                       move_group                     ros2_control
   │                               │                              │
   ├─ set_goal_state(pose) ────────►                              │
   │                               ├─ IK: pose → joint angles    │
   │                               ├─ Planning: 生成无碰路径       │
   │                               ├─ TimeParameterization:       │
   │                               │  给每个路点加上时间戳          │
   │                               │                              │
   ├─ plan() ──────────────────────►                              │
   │  ◄── plan_result (.trajectory)│                              │
   │                               │                              │
   ├─ execute(trajectory) ────────►                              │
   │                               ├─ FollowJointTrajectory       │
   │                               │   Action Goal ──────────────►│
   │                               │   {joint_names, points[      │
   │                               │     {positions, velocities,  │
   │                               │      accelerations,          │
   │                               │      time_from_start}]}      │
   │                               │                              ├─ 插值到控制周期
   │                               │                              ├─ 写 position cmd
   │                               │                              ├─ 读 joint state
   │                               │                              │
   │                               │  ◄── Action Feedback ────────┤
   │                               │      {actual positions,       │
   │                               │       error, ...}            │
   │                               │                              │
   │                               │  ◄── Action Result ─────────┤
   │                               │      {SUCCESS/FAILURE}       │
   │  ◄── execution complete       │                              │
```

### 5.2 轨迹消息结构

`FollowJointTrajectory` action 发送的 Goal 是一个 `trajectory_msgs/JointTrajectory`：

```yaml
joint_names: [shoulder_roll_l_joint, shoulder_yaw_l_joint, elbow_pitch_l_joint, ...]
points:
  - positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    time_from_start: {sec: 0, nanosec: 0}
  - positions: [0.01, 0.02, 0.001, 0.0, 0.0, 0.0]
    velocities: [0.1, 0.2, 0.01, 0.0, 0.0, 0.0]
    time_from_start: {sec: 0, nanosec: 100000000}   # 0.1 秒
  - positions: [0.05, 0.08, 0.005, 0.0, 0.0, 0.0]
    velocities: [0.2, 0.4, 0.02, 0.0, 0.0, 0.0]
    time_from_start: {sec: 0, nanosec: 200000000}   # 0.2 秒
  # ...更多路径点...
```

**关键点：**
- `joint_names` 顺序与 `positions` 一一对应
- `time_from_start` 指定每个路点应该何时到达
- 控制器在相邻路点之间按时间线性插值
- 如果你有**直接控制关节的 ROS 接口**（比如 CAN 总线驱动板），你可以：
  - **写一个 ROS 2 节点**读取 `/joint_trajectory_controller/follow_joint_trajectory/goal` 话题
  - 将轨迹点逐点发给你的硬件驱动
  - 回传关节状态到 `/joint_states`

### 5.3 模拟 vs 真机切换

目前你的配置是**仿真模式** (`mock_components/GenericSystem`)。

**切换到真机的步骤：**

**第 1 步：** 修改 `tianyi2_complete.ros2_control.xacro`，将 `<plugin>` 改为你真实的硬件接口：
```xml
<!-- 仿真：FakeSystem -->
<plugin>mock_components/GenericSystem</plugin>

<!-- 真机（示例）：你需要实现自己的 HardwareInterface -->
<!-- ros2_control hardware_interface/YourRobotHardwareInterface -->
```

**第 2 步：** 修改 `ros2_controllers.yaml`，command_interface 可能需要改为 velocity 或 effort 模式：
```yaml
hand_L_Arn_Group_controller:
  ros__parameters:
    command_interfaces:
      - velocity          # 如果硬件支持速度环
    # - effort            # 如果硬件支持力矩环
```

**第 3 步：** 确认 `moveit_controllers.yaml` 中 controller name 和 joints 与真机一致。

**第 4 步：** 测试通信链：
```bash
# 检查 FollowJointTrajectory action 是否可达
ros2 action list | grep follow_joint_trajectory

# 确认 joint_states 话题有数据
ros2 topic echo /joint_states
```

---

## 6. 实用技巧

### 6.1 设置速度和加速度

在 `plan()` 之前：
```python
# 通过 single_plan_parameters 设置
from moveit.planning import SinglePlanRequestParameters

params = SinglePlanRequestParameters()
params.max_velocity_scaling_factor = 0.5    # 50% 速度
params.max_acceleration_scaling_factor = 0.5

left_arm.plan(single_plan_parameters=params)
```

也可在 `joint_limits.yaml` 中设置全局默认值（你目前是 `0.1`）。

### 6.2 查看规划的路径在 RViz 中

`execute()` 前，轨迹自动发布到 `/display_planned_path` 话题，RViz 中的 `MotionPlanning` 面板会显示。

### 6.3 调试 IK 是否可达

```python
# 如果 pose 目标 IK 无解，plan() 直接返回失败
if not plan_result:
    # 可能原因：
    # ① 目标超出工作空间
    # ② kinematics_solver_timeout 太短 → 改为 0.05
    # ③ kinematics_solver_search_resolution 太粗 → 改为 0.001
```

### 6.4 添加碰撞对象

```python
from moveit.planning import (
    tianyi.get_planning_scene_monitor().write_only().add_box(...)
)
# 或用 pymoveit2 的 add_collision_box()
```

### 6.5 避开奇点

- 肘部伸直（`elbow_pitch ≈ 0`）是典型奇点 — 避免
- 手腕 `roll ≈ ±π` 也可能导致万向节锁 — 用 quaternion

---

## 7. 当前项目的差距和改进方向

| 当前状态 | 建议 |
|---|---|
| 只有左臂规划组 | SRDF 添加 `hand_R_Arn_Group`（右侧镜像链） |
| KDL IK 求解器 | 如遇 IK 收敛问题可换 `trac_ik` 或 `bio_ik` |
| 仿真 FakeSystem | 写真实 HardwareInterface 驱动 CAN 总线 |
| 无 Cartesian API 绑定 | moveit_py 的 `compute_cartesian_path` 需检查版本；当前可用 Pilz LIN |
| velocity_factor=0.1 | 调试稳定后调高到 0.5-1.0 |
| 无碰撞场景物体 | 业务逻辑中 add scene objects（桌子、工具） |

---

## 参考资料

- [MoveIt 2 官方文档](https://moveit.picknik.ai/main)
- [Motion Planning Python API 教程](https://moveit.picknik.ai/main/doc/examples/motion_planning_python_api/motion_planning_python_api_tutorial.html)
- [Move Group C++ Interface 教程](https://moveit.picknik.ai/main/doc/examples/move_group_interface/move_group_interface_tutorial.html)
- [Controller Configuration 教程](https://moveit.picknik.ai/main/doc/examples/controller_configuration/controller_configuration_tutorial.html)
- [并行规划教程](https://moveit.picknik.ai/main/doc/how_to_guides/parallel_planning/parallel_planning_tutorial.html)
- [MoveIt 概念：move_group 节点](https://moveit.picknik.ai/main/doc/concepts/move_group.html)
