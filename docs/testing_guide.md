# 测试步骤

## 1. 仿真测试

### 启动全流程

```bash
source install/setup.bash

# 终端 1：启动 MoveIt 仿真（move_group + RViz + ros2_control）
ros2 launch tianti2_moveit demo.launch.py

# 终端 2：启动位姿控制节点
ros2 run tianyi2_control pose_goal_node
```

### 控制话题速查

| 话题 | 功能 |
|------|------|
| `/left_arm/target_pose` | 左臂绝对位姿 (base系) |
| `/left_arm/delta_pose` | 左臂相对变换 (平移base系 + 旋转base系) |
| `/right_arm/target_pose` | 右臂绝对位姿 (base系) |
| `/right_arm/delta_pose` | 右臂相对变换 (平移base系 + 旋转base系) |

### 发送位姿目标

#### 左臂

```bash
# ── 绝对位姿 ──
ros2 topic pub /left_arm/target_pose geometry_msgs/msg/PoseStamped "
pose:
  position: {x: 0.404, y: 0.453, z: 1.031}
  orientation: {x: -0.453, y: 0.575, z: -0.467, w: 0.497}
" -1


# ── 相对旋转：末端绕 base 系 X 轴 roll -90° ──
ros2 topic pub /left_arm/delta_pose geometry_msgs/msg/PoseStamped "
pose:
  position: {x: 0.0, y: 0.0, z: 0.0}
  orientation: {w: 0.707, x: -0.707, y: 0.0, z: 0.0}
" -1  
  


```

#### 右臂

```bash
# ── 绝对位姿 ──
ros2 topic pub /right_arm/target_pose geometry_msgs/msg/PoseStamped "
pose:
  position: {x: 0.376, y: -0.624, z: 0.911}
  orientation: {x: 0.477, y: 0.520, z: 0.479, w: 0.522}
" -1

# ── 相对旋转：末端绕 base 系 X 轴 roll -90° ──
ros2 topic pub /right_arm/delta_pose geometry_msgs/msg/PoseStamped "
pose:
  position: {x: 0.0, y: 0.0, z: 0.0}
  orientation: {w: 0.707, x: -0.707, y: 0.0, z: 0.0}
" -1
```


## 2. 真机测试

### 前提

- 机器人已开机，SDK 已启动（`/arm/status`、`/head/status`、`/leg/status` 等话题可用）
- 机器人 SDK 已 source：`source /home/storm/ros2ws/install/setup.bash`

### 数据流

```
pose_goal_node  →  move_group  →  /L_Arm_Group_controller/joint_trajectory (action)
                                      │
                                ros2_control (FakeSystem 执行)
                                      │
                                /L_Arm_Group_controller/state
                                /R_Arm_Group_controller/state
                                      │
                                bridge_node 订阅
                                      │
                              /arm/cmd_pos
                              (CmdSetMotorPosition)
                                      │
                               机器人 body_control


/arm|head|leg/status  →  joint_state_publisher  →  /joint_states  →  move_group (真实状态规划)
```

### 启动流程

```bash
source ~/ros2ws/install/setup.bash
source ~/moveit2_demo_ws/install/setup.bash

# 终端 1：启动 MoveIt 真机控制（不含 bridge_node）
ros2 launch tianyi2_control real_control.launch.py

# 可选：关闭 RViz
# ros2 launch tianyi2_control real_control.launch.py use_rviz:=false

# 终端 2：启动桥接节点（控制器状态 → /arm/cmd_pos）
ros2 run tianyi2_control bridge_node

# 终端 3：启动位姿控制节点
ros2 run tianyi2_control pose_goal_node
```

> **注意**：`real_control.launch.py` 与 `demo.launch.py` 的区别：
> - `/joint_states` 由 `joint_state_publisher` 从真实硬件状态发布（非 FakeSystem）
> - 不启动 `joint_state_broadcaster`
> - 不启动 `bridge_node`（需单独启动）

### 发送控制命令

与仿真相同：
```bash
# 左臂绝对位姿
ros2 topic pub /left_arm/target_pose geometry_msgs/msg/PoseStamped "
pose:
  position: {x: 0.404, y: 0.453, z: 1.031}
  orientation: {x: -0.453, y: 0.575, z: -0.467, w: 0.497}
" -1

# 右臂相对旋转 roll -90°
ros2 topic pub /right_arm/delta_pose geometry_msgs/msg/PoseStamped "
pose:
  position: {x: 0.0, y: 0.0, z: 0.0}
  orientation: {w: 0.707, x: -0.707, y: 0.0, z: 0.0}
" -1

ros2 topic pub /right_arm/delta_pose geometry_msgs/msg/PoseStamped "
pose:
  position: {x: 0.1, y: 0.1, z: 0.1}
  orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}
" -1
```

---

## 3. 常用调试命令

```bash
# 查看所有 TF frame
timeout 3 ros2 topic echo /tf --once | grep child_frame_id | sort -u

# 查看 /joint_states 包含哪些关节
timeout 2 ros2 topic echo /joint_states --once | grep -A20 "name:"

# 查看真实关节状态（需 source 机器人 SDK）
source /home/storm/ros2ws/install/setup.bash
timeout 2 ros2 topic echo /arm/status --once --field status
timeout 2 ros2 topic echo /head/status --once --field status
timeout 2 ros2 topic echo /leg/status --once --field status

# 查看控制器实时目标位置
ros2 topic echo /L_Arm_Group_controller/state --field desired --once
ros2 topic echo /R_Arm_Group_controller/state --field desired --once

# 查看桥接节点发布的电机指令
ros2 topic echo /arm/cmd_pos

# 检查双臂 TF 链完整性
ros2 run tf2_ros tf2_echo base L_base_link
ros2 run tf2_ros tf2_echo base R_base_link

# 检查相机 TF
ros2 run tf2_ros tf2_echo base camera_head_link

# 查看 move_group 服务列表
ros2 service list | grep -E "plan|kinematic|cartesian|ik"

# 查看 action 列表
ros2 action list
```
