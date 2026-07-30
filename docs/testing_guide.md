# 测试步骤

## 1. 仿真测试

### 启动全流程

```bash
source install/setup.bash

# 终端 1：启动 MoveIt 仿真（move_group + RViz + ros2_control）
ros2 launch demo demo.launch.py

# 终端 2：启动位姿控制节点
ros2 run tianyi2_control pose_goal_node

# 终端 3：验证 TF 链完整
timeout 8 bash -c 'ros2 run tf2_ros tf2_echo base L_base_link 2>&1' | head -10
```

### 验证 TF 输出示例

```
- Translation: [0.049, 0.272, 0.418]
- Rotation: in Quaternion (xyzw) [0.706, -0.706, -0.031, -0.030]
```

出现以上数据即表示 TF 链完整。

### 发送位姿目标

回到当前位置（不会动，仅验证链路）：

```bash
# ── 绝对位姿：直接移动到 base 系下的目标 ──
ros2 topic pub /pose_goal_node/target_pose geometry_msgs/msg/PoseStamped "
pose:
  position: {x: 0.049, y: 0.272, z: 0.418}
  orientation: {x: -0.706, y: 0.706, z: 0.031, w: 0.030}
" -1

# 抬手
ros2 topic pub /pose_goal_node/target_pose geometry_msgs/msg/PoseStamped "
pose:
  position: {x: 0.404, y: 0.453, z: 1.031}
  orientation: {x: -0.453, y: 0.575, z: -0.467, w: 0.497}
" -1

# ── 相对变换：在末端当前位置上叠加（全部 base 系）──
# 旋转 delta * current
ros2 topic pub /pose_goal_node/delta_pose geometry_msgs/msg/PoseStamped "
pose:
  orientation: {w: 0.707, x: -0.707, y: 0.0, z: 0.0}
" -1

" -1
```

### 观察日志

终端 2 应输出：

```
[pose_goal_node]: Target: x=0.049 y=0.272 z=0.418
[pose_goal_node]: Plan OK: N waypoints, duration=X.XXs
[pose_goal_node]: Sending execute goal...
[pose_goal_node]: Executing...
[pose_goal_node]: Execute done
```

---

## 2. 真机测试

### 启动全流程

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

# 查看 move_group 服务列表
ros2 service list | grep -E "plan|kinematic|cartesian|ik"

# 查看 action 列表
ros2 action list

# 测试单次规划（绕过控制节点，直接调 move_group）
ros2 service call /plan_kinematic_path moveit_msgs/srv/GetMotionPlan "..."
```
