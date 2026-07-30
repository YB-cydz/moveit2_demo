# 测试步骤

## 1. 仿真测试

### 启动全流程

```bash
source install/setup.bash

# 终端 1：启动 MoveIt 仿真（move_group + RViz + ros2_control）
ros2 launch demo demo.launch.py

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

- 机器人已开机，SDK 已启动（`/joint_states` + `/tf` 完整）
- 机器人 SDK 已 source：`source /home/storm/ros2ws/install/setup.bash`

### 数据流

```
pose_goal_node  →  move_group  →  /display_planned_path
                                      │
                                bridge_node 订阅
                                      │
                              /arm/cmd_pos
                              (CmdSetMotorPosition)
                                      │
                               机器人 body_control
```

### 启动流程

```bash
source ~/ros2ws/install/setup.bash
source ~/moveit2_demo_ws/install/setup.bash

# 终端 1：启动 MoveIt
ros2 launch demo demo.launch.py

# 终端 2：启动桥接节点（将轨迹转为 /arm/cmd_pos）
ros2 run tianyi2_control bridge_node

# 终端 3：启动位姿控制节点
ros2 run tianyi2_control pose_goal_node
```

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
