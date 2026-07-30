# 测试步骤

## 1. 仿真测试

### 启动全流程

```bash
source install/setup.bash

# 终端 1：启动 MoveIt 仿真（move_group + RViz + ros2_control）
ros2 launch demo demo.launch.py

# 终端 2：启动位姿控制节点
ros2 run tianyi2_control pose_goal_node

# 终端 3：验证 TF 链完整（双臂）
timeout 8 ros2 run tf2_ros tf2_echo base L_base_link 2>&1 | head -6
timeout 8 ros2 run tf2_ros tf2_echo base R_base_link 2>&1 | head -6
```

### 控制话题速查

| 话题 | 功能 |
|------|------|
| `/pose_goal_node/left_arm/target_pose` | 左臂绝对位姿 (base系) |
| `/pose_goal_node/left_arm/delta_pose` | 左臂相对变换 (平移base系 + 旋转末端系) |
| `/pose_goal_node/right_arm/target_pose` | 右臂绝对位姿 (base系) |
| `/pose_goal_node/right_arm/delta_pose` | 右臂相对变换 (平移base系 + 旋转末端系) |

### 发送位姿目标

```bash
# ── 左臂绝对位姿 ──
ros2 topic pub /pose_goal_node/left_arm/target_pose geometry_msgs/msg/PoseStamped "
pose:
  position: {x: 0.404, y: 0.453, z: 1.031}
  orientation: {x: -0.453, y: 0.575, z: -0.467, w: 0.497}
" -1

# ── 右臂绝对位姿 ──
ros2 topic pub /pose_goal_node/right_arm/target_pose geometry_msgs/msg/PoseStamped "
pose:
  position: {x: -0.404, y: 0.453, z: 1.031}
  orientation: {x: -0.453, y: 0.575, z: -0.467, w: 0.497}
" -1

# ── 左臂相对变换：末端 roll -90°（绕自身 X 轴旋转）──
ros2 topic pub /pose_goal_node/left_arm/delta_pose geometry_msgs/msg/PoseStamped "
pose:
  position: {x: 0.0, y: 0.0, z: 0.0}
  orientation: {w: 0.707, x: -0.707, y: 0.0, z: 0.0}
" -1

# ── 左臂相对变换：base 系沿 X 平移 10cm ──
ros2 topic pub /pose_goal_node/left_arm/delta_pose geometry_msgs/msg/PoseStamped "
pose:
  position: {x: 0.1, y: 0.0, z: 0.0}
  orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}
" -1
```

### 观察日志

终端 2 应输出：

```
[L_Arm_Group] [ABS] x=0.404 y=0.453 z=1.031
[L_Arm_Group] Plan OK: N waypoints, duration=X.XXs
[L_Arm_Group] Sending execute goal...
[L_Arm_Group] Executing...
[L_Arm_Group] Execute done
```

---

## 2. 真机测试

```bash
# 前提：机器人驱动节点已在运行（/joint_states + /tf 完整）

# 终端 1：启动 MoveIt（仅 move_group + RViz，不含仿真）
ros2 launch demo real_robot.launch.py

# 终端 2：启动桥接节点（待开发）
# ros2 run tianyi2_control bridge_node

# 终端 3：启动位姿控制节点
ros2 run tianyi2_control pose_goal_node
```

---

## 3. 常用调试命令

```bash
# 查看所有 TF frame
timeout 3 ros2 topic echo /tf --once | grep child_frame_id | sort -u

# 查看 /joint_states 包含哪些关节
timeout 2 ros2 topic echo /joint_states --once | grep -A20 "name:"

# 检查双臂 TF 链完整性
ros2 run tf2_ros tf2_echo base L_base_link
ros2 run tf2_ros tf2_echo base R_base_link

# 查看 move_group 服务列表
ros2 service list | grep -E "plan|kinematic|cartesian|ik"

# 查看 action 列表
ros2 action list
```
