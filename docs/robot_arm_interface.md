# 天轶2.0 手臂控制接口

## 话题

| 话题 | 类型 | 方向 |
|------|------|------|
| `/arm/cmd_pos` | `bodyctrl_msgs/msg/CmdSetMotorPosition` | Python → 机器人 |

## 消息定义

### CmdSetMotorPosition

```python
# 来自 bodyctrl_msgs
std_msgs/Header header
SetMotorPosition[] cmds
```

### SetMotorPosition

```python
uint16 name   # 电机编号 (MotorName)
float32 pos   # 目标位置 (rad)
float32 spd   # 目标转速 (rpm)
float32 cur   # 目标电流 (A)
```

## 电机编号

每臂 7 个电机，MoveIt 每臂只控制前 6 个（第 7 个为手指）。

### 左臂 (11–16)

| 电机 ID | MoveIt 关节 |
|:-------:|------------|
| 11 | shoulder_roll_l_joint |
| 12 | shoulder_yaw_l_joint |
| 13 | elbow_pitch_l_joint |
| 14 | elbow_yaw_l_joint |
| 15 | wrist_pitch_l_joint |
| 16 | wrist_roll_l_joint |
| 17 | (手指，不在 MoveIt 中) |

### 右臂 (21–26)

| 电机 ID | MoveIt 关节 |
|:-------:|------------|
| 21 | shoulder_roll_r_joint |
| 22 | shoulder_yaw_r_joint |
| 23 | elbow_pitch_r_joint |
| 24 | elbow_yaw_r_joint |
| 25 | wrist_pitch_r_joint |
| 26 | wrist_roll_r_joint |
| 27 | (手指，不在 MoveIt 中) |

## 桥接代码（核心逻辑）

```python
from bodyctrl_msgs.msg import CmdSetMotorPosition, SetMotorPosition

# 电机映射
LEFT_MOTORS  = [11, 12, 13, 14, 15, 16]
RIGHT_MOTORS = [21, 22, 23, 24, 25, 26]

def build_cmd(left_positions, right_positions):
    """
    left_positions: 左臂 6 个关节角度 (rad)，来自 MoveIt trajectory
    right_positions: 右臂 6 个关节角度 (rad)，来自 MoveIt trajectory
    """
    cmd = CmdSetMotorPosition()
    cmd.header.stamp = node.get_clock().now().to_msg()

    for motor_id, pos in zip(LEFT_MOTORS, left_positions):
        sp = SetMotorPosition()
        sp.name = motor_id
        sp.pos = pos
        sp.spd = 0.0
        sp.cur = 0.0
        cmd.cmds.append(sp)

    for motor_id, pos in zip(RIGHT_MOTORS, right_positions):
        sp = SetMotorPosition()
        sp.name = motor_id
        sp.pos = pos
        sp.spd = 0.0
        sp.cur = 0.0
        cmd.cmds.append(sp)

    return cmd
```

## 完整桥接流程

```
MoveIt trajectory (JointTrajectory)
  ├── 左臂 joint_names + positions
  └── 右臂 joint_names + positions
              │
       bridge_node 映射
              │
         /arm/cmd_pos
         (CmdSetMotorPosition)
              │
       机器人 body_control
              │
          电机执行
```
