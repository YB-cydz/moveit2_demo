# pymoveit2 使用手册

> 基于 [AndrejOrsula/pymoveit2](https://github.com/AndrejOrsula/pymoveit2)，适用于 ROS 2 Humble/Iron。

## 目录

1. [模块总览](#1-模块总览)
2. [MoveIt2 — 机械臂运动控制](#2-moveit2--机械臂运动控制)
3. [MoveIt2Servo — 实时伺服控制](#3-moveit2servo--实时伺服控制)
4. [MoveIt2Gripper — 夹爪控制（JointTrajectoryController）](#4-moveit2gripper--夹爪控制jointtrajectorycontroller)
5. [GripperCommand — 夹爪控制（GripperCommand Action）](#5-grippercommand--夹爪控制grippercommand-action)
6. [GripperInterface — 自适应夹爪接口](#6-gripperinterface--自适应夹爪接口)
7. [高级用法](#7-高级用法)
8. [在天轶2.0 上使用](#8-在天轶20-上使用)
9. [常见问题与注意事项](#9-常见问题与注意事项)

---

## 1. 模块总览

```
pymoveit2/
├── moveit2.py              # MoveIt2 — 机械臂运动规划与执行
├── moveit2_servo.py        # MoveIt2Servo — 实时笛卡尔伺服
├── moveit2_gripper.py      # MoveIt2Gripper — 基于 JointTrajectoryController 的夹爪
├── gripper_command.py      # GripperCommand — 基于 GripperCommand Action 的夹爪
├── gripper_interface.py    # GripperInterface — 自动选择夹爪控制方式
├── utils.py                # enum_to_str — ROS 2 枚举到字符串转换
└── robots/                 # 机器人预设（目前仅有 Panda）
```

顶层导出：`MoveIt2`, `MoveIt2State`, `MoveIt2Servo`, `MoveIt2Gripper`, `GripperCommand`, `GripperInterface`

---

## 2. MoveIt2 — 机械臂运动控制

这是 pymoveit2 的核心类，封装了 MoveIt 2 的规划（planning）、执行（execution）、正运动学（FK）、逆运动学（IK）、以及碰撞场景管理。

### 2.1 初始化

```python
from pymoveit2 import MoveIt2

moveit2 = MoveIt2(
    node=node,                                    # rclpy.node.Node 实例
    joint_names=[                                 # 规划组关节名列表（按顺序）
        "shoulder_roll_l_joint",
        "shoulder_yaw_l_joint",
        "elbow_pitch_l_joint",
        "elbow_yaw_l_joint",
        "wrist_pitch_l_joint",
        "wrist_roll_l_joint",
    ],
    base_link_name="base",                        # 基座坐标系
    end_effector_name="L_base_link",              # 末端执行器坐标系
    group_name="hand_L_Arn_Group",                # MoveIt 规划组名
    use_move_group_action=False,                   # 是否使用 MoveGroup Action（合并 plan+execute）
    ignore_new_calls_while_executing=False,        # 执行中是否忽略新指令
    callback_group=callback_group,                 # 可选，ROS 2 回调组
)
```

#### 两种执行模式

| 参数 | 含义 | 适用场景 |
|---|---|---|
| `use_move_group_action=False`（默认） | `plan()` + `execute()` 分离调用 | 需要检查或修改规划结果后再执行 |
| `use_move_group_action=True` | `move_to_pose()` / `move_to_configuration()` 一步完成 | 简单的规划+执行流程 |

### 2.2 运动规划

#### plan() — 同步规划

```python
# 关节空间规划（关节目标）
trajectory = moveit2.plan(
    joint_positions=[1.36, 1.46, -0.96, 0.77, -0.01, -0.25],
    joint_names=None,           # 可指定部分关节名
    tolerance_joint_position=0.001,
    weight_joint_position=1.0,
    start_joint_state=None,     # 起始关节状态，默认当前状态
)
# 返回 Optional[JointTrajectory]，失败返回 None

# 笛卡尔空间规划（位姿目标）
trajectory = moveit2.plan(
    position=[0.3, 0.0, 0.5],
    quat_xyzw=[0.0, 0.0, 0.0, 1.0],
    frame_id="base",            # 参考坐标系，默认 base_link_name
    target_link=None,           # 目标 link，默认 end_effector_name
    tolerance_position=0.001,
    tolerance_orientation=0.001,
    weight_position=1.0,
    weight_orientation=1.0,
)

# 笛卡尔路径规划 （直线路径）
trajectory = moveit2.plan(
    position=[0.3, 0.0, 0.5],
    quat_xyzw=[0.0, 0.0, 0.0, 1.0],
    cartesian=True,
    max_step=0.0025,            # 笛卡尔路径最大步长
    cartesian_fraction_threshold=0.0,  # 路径完成比例阈值（如 0.9 表示至少完成 90%）
)
```

#### plan_async() — 异步规划

```python
future = moveit2.plan_async(position=[0.3, 0.0, 0.5], quat_xyzw=[...])

# 等待完成并获取轨迹
while not future.done():
    rclpy.spin_once(node, timeout_sec=1.0)

trajectory = moveit2.get_trajectory(future, cartesian=False)
```

### 2.3 运动执行

#### 两步法：plan + execute

```python
trajectory = moveit2.plan(joint_positions=[...])
if trajectory is not None:
    moveit2.execute(trajectory)
    success = moveit2.wait_until_executed()  # 阻塞等待，返回 bool
```

#### 一步法：move_to_pose / move_to_configuration

```python
# 移动到位姿
moveit2.move_to_pose(
    position=[0.3, 0.0, 0.5],
    quat_xyzw=[0.0, 0.0, 0.0, 1.0],
    frame_id="base",
    tolerance_position=0.001,
    tolerance_orientation=0.001,
    cartesian=False,            # True=笛卡尔直线路径
    cartesian_max_step=0.0025,
    cartesian_fraction_threshold=0.0,
)

# 移动到关节构型
moveit2.move_to_configuration(
    joint_positions=[1.36, 1.46, -0.96, 0.77, -0.01, -0.25],
    joint_names=None,           # 可指定部分关节名
    tolerance=0.001,
    weight=1.0,
)
moveit2.wait_until_executed()
```

#### 异步执行监控

```python
from pymoveit2 import MoveIt2State

moveit2.move_to_pose(position=[...], quat_xyzw=[...])

# 轮询状态
while moveit2.query_state() == MoveIt2State.REQUESTING:
    print("等待目标被接受...")
    rclpy.spin_once(node, timeout_sec=0.1)

# 获取执行 Future
future = moveit2.get_execution_future()
# 可取消
moveit2.cancel_execution()
# 等待结果
while not future.done():
    rclpy.spin_once(node, timeout_sec=0.1)
print("结果:", future.result().status)
```

#### MoveIt2State 枚举

| 状态 | 含义 |
|---|---|
| `IDLE` | 空闲，无运动 |
| `REQUESTING` | 已发送请求，等待被接受 |
| `EXECUTING` | 正在执行 |

#### 复位控制器（仅仿真）

```python
# 瞬时跳转到指定关节构型（仿真专用）
moveit2.reset_controller(joint_state=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
```

### 2.4 运动学计算

#### 逆运动学 IK

```python
# 同步
joint_state = moveit2.compute_ik(
    position=[0.3, 0.0, 0.5],
    quat_xyzw=[0.0, 0.0, 0.0, 1.0],
    ik_link_name=None,          # 默认末端执行器
    start_joint_state=None,     # 搜索起始点（种子），默认当前状态
    constraints=None,           # 额外约束
)
# 返回 Optional[JointState]

# 异步
future = moveit2.compute_ik_async(position=[...], quat_xyzw=[...])
while not future.done():
    rclpy.spin_once(node, timeout_sec=0.1)
joint_state = moveit2.get_compute_ik_result(future)
```

#### 正运动学 FK

```python
# 同步
pose = moveit2.compute_fk(
    joint_state=[1.36, 1.46, -0.96, 0.77, -0.01, -0.25],
    fk_link_names=["L_base_link"],      # None=仅末端执行器，list=多个 link
)
# fk_link_names=None → 返回单个 PoseStamped
# fk_link_names=[...] → 返回 List[PoseStamped]

# 异步
future = moveit2.compute_fk_async(joint_positions=[...], fk_link_names=[...])
while not future.done():
    rclpy.spin_once(node, timeout_sec=0.1)
pose = moveit2.get_compute_fk_result(future, fk_link_names=[...])
```

### 2.5 约束管理

#### 目标约束

`set_pose_goal()` 是 `set_position_goal()` + `set_orientation_goal()` 的组合。这些设置后需要调用 `plan()` 或 `move_to_pose()` 才会生效。

```python
# 位置约束
moveit2.set_position_goal(
    position=(0.3, 0.0, 0.5),
    frame_id="base",            # 参考坐标系
    target_link=None,           # 默认末端执行器
    tolerance=0.001,            # 球体容差半径
    weight=1.0,
)

# 姿态约束
moveit2.set_orientation_goal(
    quat_xyzw=(0.0, 0.0, 0.0, 1.0),
    frame_id="base",
    target_link=None,
    tolerance=0.001,            # 或 (x_tol, y_tol, z_tol)
    weight=1.0,
    parameterization=0,         # 0=Euler, 1=Rotation Vector
)

# 关节约束
moveit2.set_joint_goal(
    joint_positions=[1.36, 1.46, ...],
    joint_names=None,           # 默认全部规划组关节
    tolerance=0.001,
    weight=1.0,
)

# 多目标约束组（可用于定义多个 IK 候选）
moveit2.create_new_goal_constraint()   # 追加一组新约束

# 清除所有目标约束
moveit2.clear_goal_constraints()
```

#### 路径约束

路径约束限制整个轨迹上的运动，不同于目标约束只限制终点。

```python
# 关节路径约束
moveit2.set_path_joint_constraint(
    joint_positions=[0.0, ...],
    tolerance=0.001,
    weight=1.0,
)

# 位置路径约束
moveit2.set_path_position_constraint(
    position=(0.3, 0.0, 0.5),
    tolerance=0.001,
    weight=1.0,
)

# 姿态路径约束（如保持水杯不洒）
moveit2.set_path_orientation_constraint(
    quat_xyzw=(0.0, 0.0, 0.0, 1.0),
    tolerance=(0.1, 0.1, 0.1),  # 绕各轴容差
    weight=1.0,
    parameterization=1,          # Rotation Vector
)

# 清除路径约束
moveit2.clear_path_constraints()
```

#### 工作空间限制

```python
moveit2.set_workspace_parameters(
    min_corner=(-1.0, -1.0, -1.0),
    max_corner=(1.0, 1.0, 1.0),
    frame_id="base",
)
```

### 2.6 碰撞对象管理

#### 添加基础几何体

```python
# 长方体
moveit2.add_collision_box(
    id="box1",
    size=(0.2, 0.1, 0.05),      # (x, y, z)
    position=(0.5, 0.0, 0.3),
    quat_xyzw=(0.0, 0.0, 0.0, 1.0),
    frame_id="base",
)

# 球体
moveit2.add_collision_sphere(
    id="ball1",
    radius=0.05,
    position=(0.5, 0.0, 0.3),
)

# 圆柱体（dimensions=[height, radius]）
moveit2.add_collision_cylinder(
    id="cylinder1",
    height=0.2,
    radius=0.04,
    position=(0.5, 0.0, 0.3),
    quat_xyzw=(0.0, 0.0, 0.0, 1.0),
)

# 圆锥体（dimensions=[height, radius]）
moveit2.add_collision_cone(
    id="cone1",
    height=0.1,
    radius=0.05,
    position=(0.5, 0.0, 0.3),
    quat_xyzw=(0.0, 0.0, 0.0, 1.0),
)
```

#### 添加网格碰撞对象（需要 trimesh）

```python
# 从文件加载
moveit2.add_collision_mesh(
    filepath="/path/to/mesh.stl",
    id="my_mesh",
    position=(0.5, 0.0, 0.3),
    quat_xyzw=(0.0, 0.0, 0.0, 1.0),
    scale=1.0,                  # 或 (sx, sy, sz)
)

# 直接传入 trimesh 对象（避免重复加载）
import trimesh
mesh = trimesh.load("/path/to/mesh.stl")
moveit2.add_collision_mesh(
    filepath=None,              # 与 mesh 互斥
    id="my_mesh",
    position=(0.5, 0.0, 0.3),
    quat_xyzw=(0.0, 0.0, 0.0, 1.0),
    mesh=mesh,
)
```

#### 移动、移除与附着

```python
# 移动碰撞对象
moveit2.move_collision(id="box1", position=(0.6, 0.0, 0.3), quat_xyzw=(0,0,0,1))

# 删除碰撞对象
moveit2.remove_collision_object(id="box1")

# 附着到机器人（如抓取物体）
moveit2.attach_collision_object(
    id="box1",
    link_name="L_base_link",    # 附着的 link
    touch_links=[],             # 允许接触的 link 列表
    weight=0.0,
)

# 解除附着
moveit2.detach_collision_object(id="box1")
moveit2.detach_all_collision_objects()

# 清空整个规划场景
future = moveit2.clear_all_collision_objects()
```

### 2.7 碰撞规则管理

```python
# 允许/禁止与某对象碰撞
future = moveit2.allow_collisions(id="box1", allow=True)
# 返回 Future，可通过 process_allow_collision_future 获取结果
```

### 2.8 规划参数

| 属性 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `max_velocity` | float | 0.0 | 最大速度缩放因子（0.0~1.0） |
| `max_acceleration` | float | 0.0 | 最大加速度缩放因子（0.0~1.0） |
| `num_planning_attempts` | int | 5 | 规划重试次数 |
| `allowed_planning_time` | float | 0.5 | 单次规划时间上限（秒） |
| `planner_id` | str | `""` | 规划器 ID（如 `"RRTConnectkConfigDefault"`） |
| `pipeline_id` | str | `""` | 规划管线 ID |
| `cartesian_avoid_collisions` | bool | — | 笛卡尔规划是否避碰 |
| `cartesian_jump_threshold` | float | — | 笛卡尔规划关节跳变阈值 |
| `joint_state` | JointState（只读） | — | 当前关节状态 |

```python
moveit2.max_velocity = 0.5
moveit2.max_acceleration = 0.3
moveit2.planner_id = "RRTConnectkConfigDefault"
moveit2.num_planning_attempts = 10
moveit2.allowed_planning_time = 1.0
```

### 2.9 完整最小示例

```python
from threading import Thread
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from pymoveit2 import MoveIt2

def main():
    rclpy.init()
    node = Node("my_arm_controller")

    moveit2 = MoveIt2(
        node=node,
        joint_names=["shoulder_roll_l_joint", "shoulder_yaw_l_joint",
                     "elbow_pitch_l_joint", "elbow_yaw_l_joint",
                     "wrist_pitch_l_joint", "wrist_roll_l_joint"],
        base_link_name="base",
        end_effector_name="L_base_link",
        group_name="hand_L_Arn_Group",
    )

    # 必须用后台线程 spin
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    thread = Thread(target=executor.spin, daemon=True)
    thread.start()
    node.create_rate(1.0).sleep()   # 等节点就绪

    # 运动到 home 位姿
    moveit2.move_to_configuration(
        joint_positions=[1.3611, 1.4589, -0.9633, 0.7704, -0.0058, -0.2483]
    )
    moveit2.wait_until_executed()

    # 笛卡尔空间移动
    moveit2.move_to_pose(position=[0.3, 0.0, 0.5], quat_xyzw=[0.0, 0.0, 0.0, 1.0])
    moveit2.wait_until_executed()

    rclpy.shutdown()
    thread.join()
```

---

## 3. MoveIt2Servo — 实时伺服控制

`MoveIt2Servo` 是 MoveIt Servo 的薄封装，通过发布 `TwistStamped` 消息实现实时的笛卡尔空间速度控制。适合遥操作、视觉伺服等场景。

### 3.1 初始化

```python
from pymoveit2 import MoveIt2Servo

servo = MoveIt2Servo(
    node=node,
    frame_id="base",            # 速度指令的参考坐标系
    namespace="",               # servo 节点的命名空间
    linear_speed=1.0,           # 线速度缩放因子
    angular_speed=1.0,          # 角速度缩放因子
    enable_at_init=True,        # 初始化时自动 enable servo
    callback_group=None,
)
```

### 3.2 速度控制

#### 笛卡尔空间 Twist

```python
# 调用 servo() 方法
servo.servo(
    linear=(0.1, 0.0, 0.0),     # (x, y, z) 线速度（m/s），会被 linear_speed 缩放
    angular=(0.0, 0.0, 0.5),    # (roll, pitch, yaw) 角速度（rad/s），会被 angular_speed 缩放
    enable_if_disabled=True,     # 未启用时自动启用
)

# 也支持直接调用实例（等同于 servo()）
servo(linear=(0.1, 0.0, 0.0), angular=(0.0, 0.0, 0.5))
```

> **重要**：Servo 必须持续高频调用（如通过 Timer），单次调用只发送一个瞬时速度指令。

#### 关节空间 Jog

```python
servo.servo_jog(
    joint_names=("shoulder_roll_l_joint", "shoulder_yaw_l_joint"),
    velocities=(0.1, -0.2),     # rad/s
    enable_if_disabled=True,
)
```

### 3.3 启停 Servo

```python
# 启动
success = servo.enable(sync=True)     # 同步
servo.enable()                        # 异步（默认）

# 停止
success = servo.disable(sync=True)
servo.disable()
```

### 3.4 属性

| 属性 | 类型 | 说明 |
|---|---|---|
| `is_enabled` | bool（只读） | Servo 是否已启用 |
| `frame_id` | str | 指令参考坐标系 |
| `linear_speed` | float | 线速度缩放因子 |
| `angular_speed` | float | 角速度缩放因子 |

### 3.5 完整示例：画圆运动

```python
from math import sin, cos
import rclpy
from rclpy.node import Node
from pymoveit2 import MoveIt2Servo

def main():
    rclpy.init()
    node = Node("servo_circle")
    servo = MoveIt2Servo(node=node, frame_id="base")

    def circle():
        t = node.get_clock().now().nanoseconds * 1e-9
        servo(linear=(sin(t), cos(t), 0.0), angular=(0.0, 0.0, 0.0))

    node.create_timer(0.1, circle)   # 10Hz
    rclpy.spin(node)
```

---

## 4. MoveIt2Gripper — 夹爪控制（JointTrajectoryController）

`MoveIt2Gripper` 继承自 `MoveIt2`，用于通过 JointTrajectoryController 控制夹爪。适用于夹爪由独立关节组成的机器人。

### 4.1 初始化

```python
from pymoveit2 import MoveIt2Gripper

gripper = MoveIt2Gripper(
    node=node,
    gripper_joint_names=["finger_joint1", "finger_joint2"],
    open_gripper_joint_positions=[0.04, 0.04],       # 张开时的关节值
    closed_gripper_joint_positions=[0.0, 0.0],       # 闭合时的关节值
    gripper_group_name="gripper",
    skip_planning=False,              # True=跳过规划，直接发轨迹点（更快但无碰撞检测）
    skip_planning_fixed_motion_duration=0.5,  # skip_planning 模式下的运动时长
    callback_group=None,
)
```

### 4.2 操作方法

```python
gripper.open(skip_if_noop=True)       # 张开（skip_if_noop=True：已经张开则跳过）
gripper.close(skip_if_noop=True)      # 闭合
gripper.toggle()                      # 切换张/闭
gripper()                             # 同 toggle()
gripper.move_to_position(0.02)        # 移动到指定位置
gripper.wait_until_executed()         # 等待动作完成
```

### 4.3 状态查询

```python
gripper.is_open        # bool
gripper.is_closed      # bool
gripper.joint_state    # 当前关节状态
```

### 4.4 跳过规划模式（skip_planning=True）

当 `skip_planning=True` 时，`open()`/`close()` 不再调用 MoveIt 规划，而是直接构造 dummy 轨迹发送给控制器。**优点**是响应极快，**缺点**是没有碰撞检测且运动平滑性取决于控制器。适用于简单夹爪。

---

## 5. GripperCommand — 夹爪控制（GripperCommand Action）

使用 ROS 2 标准的 `control_msgs/GripperCommand` Action 接口。适用于支持此协议的夹爪控制器。

### 5.1 初始化

```python
from pymoveit2 import GripperCommand

gripper = GripperCommand(
    node=node,
    gripper_joint_names=["finger_joint"],
    open_gripper_joint_positions=0.04,
    closed_gripper_joint_positions=0.0,
    max_effort=10.0,                              # 最大夹持力
    ignore_new_calls_while_executing=True,
    gripper_command_action_name="gripper_action_controller/gripper_cmd",
)
```

### 5.2 操作方法

API 与 `MoveIt2Gripper` 完全一致：

```python
gripper.open()
gripper.close()
gripper.toggle()
gripper.move_to_position(0.02)
gripper.wait_until_executed()
gripper.is_open
gripper.is_closed
```

---

## 6. GripperInterface — 自适应夹爪接口

`GripperInterface` 同时继承 `MoveIt2Gripper` 和 `GripperCommand`，**自动检测**可用的 Action 服务并选择合适的接口。

- 如果 `gripper_command` Action 可用 → 使用 `GripperCommand`
- 否则如果 `execute_trajectory` Action 可用 → 使用 `MoveIt2Gripper`
- 否则报错

### 6.1 初始化

```python
from pymoveit2 import GripperInterface

gripper = GripperInterface(
    node=node,
    gripper_joint_names=["finger_joint1", "finger_joint2"],
    open_gripper_joint_positions=[0.04, 0.04],
    closed_gripper_joint_positions=[0.0, 0.0],
    gripper_group_name="gripper",
    max_effort=10.0,
    gripper_command_action_name="gripper_action_controller/gripper_cmd",
)
```

### 6.2 操作方法

与上述两个接口完全一致：`open()`, `close()`, `toggle()`, `move_to_position()`, `is_open`, `is_closed`。

> **推荐**：除非明确知道要用哪种控制方式，否则优先使用 `GripperInterface`。

---

## 7. 高级用法

### 7.1 约束组合 — 多目标 IK

```python
# 第一组候选：位置 A
moveit2.set_pose_goal(position=(0.3, 0.0, 0.5), quat_xyzw=(0,0,0,1))
moveit2.create_new_goal_constraint()
# 第二组候选：位置 B
moveit2.set_pose_goal(position=(0.3, 0.1, 0.5), quat_xyzw=(0,0,0,1))
moveit2.create_new_goal_constraint()
# 第三组候选：关节目标
moveit2.set_joint_goal(joint_positions=[...])

# 规划时会尝试所有约束组，返回第一组成功的
trajectory = moveit2.plan()
```

### 7.2 姿态路径约束 — 保持水平

```python
# 路径约束：末端始终保持水平（orientation 不变）
moveit2.set_path_orientation_constraint(
    quat_xyzw=(0.0, 0.0, 0.0, 1.0),
    tolerance=(3.14, 0.1, 0.1),    # roll 自由，pitch/yaw 严格
    parameterization=1,             # Rotation Vector 参数化
)
moveit2.move_to_pose(position=(0.4, -0.2, 0.3), quat_xyzw=(0,0,0,1))
```

### 7.3 从非当前状态规划

```python
# 从指定的起始关节状态规划（不影响实际机器人）
trajectory = moveit2.plan(
    position=[0.3, 0.0, 0.5],
    quat_xyzw=[0,0,0,1],
    start_joint_state=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
)
# 也可以传入 JointState 消息
trajectory = moveit2.plan(
    position=[0.3, 0.0, 0.5],
    quat_xyzw=[0,0,0,1],
    start_joint_state=some_joint_state_msg,
)
```

### 7.4 线程模型

pymoveit2 的所有 Action/Service 调用在内部是异步的，但需要在后台 spin 节点：

```python
# 标准模式
executor = rclpy.executors.MultiThreadedExecutor(2)
executor.add_node(node)
thread = Thread(target=executor.spin, daemon=True)
thread.start()
node.create_rate(1.0).sleep()  # 等待初始化
```

### 7.5 内置工具函数

```python
from pymoveit2.utils import enum_to_str
from moveit_msgs.msg import MoveItErrorCodes

# 将 MoveIt 错误码转为可读字符串
error_str = enum_to_str(MoveItErrorCodes, error_code_val)
```

---

## 8. 在天轶2.0 上使用

### 8.1 创建天轶的 Robot 预设（推荐）

参考 `pymoveit2/robots/panda.py` 的模式，可以在项目中创建天轶的预设：

```python
# my_package/tianyi2_config.py
from typing import List

MOVE_GROUP_ARM: str = "hand_L_Arn_Group"

def joint_names() -> List[str]:
    return [
        "shoulder_roll_l_joint",
        "shoulder_yaw_l_joint",
        "elbow_pitch_l_joint",
        "elbow_yaw_l_joint",
        "wrist_pitch_l_joint",
        "wrist_roll_l_joint",
    ]

def base_link_name() -> str:
    return "base"

def end_effector_name() -> str:
    return "L_base_link"
```

### 8.2 天轶的 Home 位姿

在 `tianyi2_complete.srdf` 中定义：

```python
HOME_JOINTS = {
    "elbow_pitch_l_joint":  -0.9633,
    "elbow_yaw_l_joint":    0.7704,
    "shoulder_roll_l_joint": 1.3611,
    "shoulder_yaw_l_joint": 1.4589,
    "wrist_pitch_l_joint": -0.0058,
    "wrist_roll_l_joint":  -0.2483,
}

# 按 joint_names() 顺序转为列表
HOME_POSITIONS = [
    HOME_JOINTS["shoulder_roll_l_joint"],
    HOME_JOINTS["shoulder_yaw_l_joint"],
    HOME_JOINTS["elbow_pitch_l_joint"],
    HOME_JOINTS["elbow_yaw_l_joint"],
    HOME_JOINTS["wrist_pitch_l_joint"],
    HOME_JOINTS["wrist_roll_l_joint"],
]
```

### 8.3 完整启动脚本示例

```python
#!/usr/bin/env python3
from threading import Thread
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from pymoveit2 import MoveIt2
from tianyi2_config import joint_names, base_link_name, end_effector_name, MOVE_GROUP_ARM, HOME_POSITIONS

def main():
    rclpy.init()
    node = Node("tianyi2_demo")

    moveit2 = MoveIt2(
        node=node,
        joint_names=joint_names(),
        base_link_name=base_link_name(),
        end_effector_name=end_effector_name(),
        group_name=MOVE_GROUP_ARM,
    )

    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    Thread(target=executor.spin, daemon=True).start()
    node.create_rate(1.0).sleep()

    moveit2.max_velocity = 0.3
    moveit2.max_acceleration = 0.3

    # 回 home
    moveit2.move_to_configuration(HOME_POSITIONS)
    moveit2.wait_until_executed()

    # 测试 IK
    ik = moveit2.compute_ik(position=[0.3, 0.0, 0.5], quat_xyzw=[0,0,0,1])
    if ik:
        node.get_logger().info(f"IK result: {ik.position}")

    rclpy.shutdown()
```

---

## 9. 常见问题与注意事项

### 9.1 `plan()` 返回 None

- 检查 MoveGroup 服务是否启动：`ros2 service list | grep plan_kinematic_path`
- 增大 `allowed_planning_time`
- 检查目标位姿是否在工作空间内（先用 `compute_ik` 验证）
- 用 `get_last_execution_error_code()` 查看具体错误码

### 9.2 务必后台 spin

`MoveIt2` 内部使用异步 Action/Service 调用，如果不在后台线程中 spin，回调永远不会触发，`wait_until_executed()` 会永远阻塞。

### 9.3 关节顺序

`joint_names` 的顺序必须与 URDF 中的顺序一致，否则 `plan()` 和 `execute()` 会出问题。天轶2.0 的关节顺序见 [8.1 节](#81-创建天轶的-robot-预设推荐)。

### 9.4 Servo 需要持续调用

`servo()` 是瞬时的速度指令。如果停止调用，机器人也会停止运动。用 Timer 以至少 10-20Hz 的频率持续调用。

### 9.5 碰撞网格需要 trimesh

```bash
pip install trimesh
```

### 9.6 已废弃的参数

- `execute_via_moveit` → 用 `use_move_group_action` 替代
- `follow_joint_trajectory_action_name` → 不再需要，已改用 `execute_trajectory` Action
