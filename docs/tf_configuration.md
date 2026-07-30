# TF 配置指南：仿真与真机

## 原理

TF 树由 `robot_state_publisher` 根据 **URDF** + **`/joint_states`** 发布。

```
/joint_states  ──→  robot_state_publisher  ──→  /tf
      ↑                      ↑
  关节状态              URDF（完整机器人模型）
```

**TF 链断裂的原因只有一种：`/joint_states` 不全。** `robot_state_publisher` 在 URDF 中找到某关节但 `/joint_states` 没有它，就跳过这个关节的 TF。

天轶2.0 从 `base` 到末端的完整链：

**左臂：**
```
base → first_leg_pitch → second_leg_pitch → waist_pitch → waist_yaw
  → shoulder_pitch_l → shoulder_roll_l → shoulder_yaw_l
  → elbow_pitch_l → elbow_yaw_l → wrist_pitch_l → wrist_roll_l
  → left_tcp → L_base_link
```

**右臂：**
```
base → first_leg_pitch → second_leg_pitch → waist_pitch → waist_yaw
  → shoulder_pitch_r → shoulder_roll_r → shoulder_yaw_r
  → elbow_pitch_r → elbow_yaw_r → wrist_pitch_r → wrist_roll_r
  → right_tcp → R_base_link
```

以上所有关节的值都必须出现在 `/joint_states` 中，TF 树才完整。双臂共用 base→waist_yaw 段，在 shoulder_pitch_l/r 处分叉。

---

## 仿真环境

### 问题

`ros2_control` 的 Mock 仿真只注册了 6 个手臂关节（`ros2_control.xacro`），`joint_state_broadcaster` 只发这 6 个到 `/joint_states`，TF 链断在 `shoulder_pitch_l_link` 上方。

### 解决方案

**1. `tianyi2_complete.ros2_control.xacro`** — 追加链上缺失的关节，只设 `state_interface`（不设 `command_interface`，即只发布状态，不参与控制）：

```xml
<!-- 仅发布状态，不参与控制 -->
<joint name="shoulder_pitch_l_joint">
    <state_interface name="position">
      <param name="initial_value">0.0</param>
    </state_interface>
</joint>
<!-- waist_yaw_joint, waist_pitch_joint, second_leg_pitch_joint,
     first_leg_pitch_joint, left_tcp_joint, left_hand_to_tcp 同理 -->
```

**2. `initial_positions.yaml`** — 追加对应初始值（全部为 0）。

**3. 无需改 `ros2_controllers.yaml`** — `joint_state_broadcaster` 自动发布 xacro 中所有注册过的关节。

### 启动

```bash
ros2 launch demo demo.launch.py
```

验证 TF 链：

```bash
# 左臂
timeout 8 ros2 run tf2_ros tf2_echo base L_base_link 2>&1 | head -6
# 右臂
timeout 8 ros2 run tf2_ros tf2_echo base R_base_link 2>&1 | head -6
```

---

## 真机环境

### 前提

机器人自带的驱动节点已经：
- 发布**完整的** `/joint_states`（所有关节）
- 发布**完整的** `/tf`（已通过 `robot_state_publisher` 加载 URDF）

### 不需做的事

- **不需要** `ros2_control` / Mock 仿真
- **不需要** `robot_state_publisher`（机器人已提供）

### 启动

使用真机专用 launch，仅启动 `move_group` + RViz：

```bash
ros2 launch demo real_robot.launch.py
```

### 后续对接

写桥接节点：接收 MoveIt 的 `ExecuteTrajectory` action，转为 `/arm/ctrl` 话题发给机器人驱动。

---

## 对比总结

| 项目 | 仿真 | 真机 |
|---|---|---|
| `/joint_states` 来源 | `joint_state_broadcaster`（ros2_control） | 机器人驱动节点 |
| TF 来源 | `robot_state_publisher` | 机器人已有 |
| 补全关节 | 手动编辑 `ros2_control.xacro` + `initial_positions.yaml` | 不需要（驱动已发完整数据） |
| 启动 launch | `demo.launch.py` | `real_robot.launch.py` |
| 控制链路 | ros2_control Mock → 虚拟硬件 | 桥接节点 → `/arm/ctrl` → 真实硬件 |
