# 天轶2.0 创建 MoveIt 2 控制节点全流程

> 基于 apt 安装的 `moveit_msgs`，直接调用 `move_group` service/action，无需任何第三方 wrapper。

---

## 1. 原理

`move_group` 节点暴露了标准 ROS 2 接口：

```
move_group (C++)
  ├── /plan_kinematic_path    GetMotionPlan service    → 运动规划
  ├── /compute_ik             GetPositionIK service    → 逆运动学
  ├── /compute_cartesian_path GetCartesianPath service → 笛卡尔路径
  └── /execute_trajectory     ExecuteTrajectory action → 执行轨迹
```

Python 节点直接构造 `moveit_msgs` 消息，通过 service client / action client 调用这些接口。

---

## 2. 创建功能包

```bash
cd ~/moveit2_demo_ws/src

ros2 pkg create tianyi2_control \
  --build-type ament_python \
  --dependencies rclpy std_msgs geometry_msgs trajectory_msgs moveit_msgs tf2_ros tf2_geometry_msgs \
  --description "天轶2.0 MoveIt2 控制节点（直调 move_group service/action）" \
  --license Apache-2.0
```

执行后 `package.xml` 自动包含全部依赖，无需手动修改。

---

## 3. 位姿控制节点代码

### 3.1 轻量 MoveIt 2 客户端

文件：`src/tianyi2_control/tianyi2_control/pose_goal_node.py`

```python
#!/usr/bin/env python3
"""
天轶2.0 位姿目标控制节点
直接调用 move_group 的 GetMotionPlan service + ExecuteTrajectory action

支持两种目标模式：
  - 绝对位姿: frame_id="base" → 直接移动到目标位姿
  - 末端相对旋转: frame_id="L_base_link" → 在当前位置上叠加旋转
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, TransformStamped
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import (
    MotionPlanRequest,
    MotionPlanResponse,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    JointConstraint,
)
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.action import ExecuteTrajectory
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose

# =============================================================================
# 天轶2.0 左臂参数
# =============================================================================
GROUP_NAME = "hand_L_Arn_Group"
BASE_LINK = "base"
END_EFFECTOR = "L_base_link"
JOINT_NAMES = [
    "shoulder_roll_l_joint", "shoulder_yaw_l_joint",
    "elbow_pitch_l_joint",   "elbow_yaw_l_joint",
    "wrist_pitch_l_joint",   "wrist_roll_l_joint",
]

PLANNER_ID = "RRTConnectkConfigDefault"
PLANNING_TIME = 5.0
PLANNING_ATTEMPTS = 10
VELOCITY_SCALE = 0.5
ACCELERATION_SCALE = 0.5


class MoveIt2Client:
    """
    轻量 MoveIt 2 客户端，封装 move_group service/action。

    用法:
        client = MoveIt2Client(node)
        traj = client.plan_pose(x=0.3, y=0.0, z=0.5, qw=1.0)
        client.execute(traj)
    """

    def __init__(self, node: Node):
        self._node = node
        self._logger = node.get_logger()

        # 规划 service client
        self._plan_client = node.create_client(
            GetMotionPlan, "/plan_kinematic_path"
        )
        # 执行 action client
        self._exec_client = ActionClient(
            node, ExecuteTrajectory, "/execute_trajectory"
        )

        # TF 监听（用于末端相对旋转）
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, node)

        self._logger.info("Waiting for /plan_kinematic_path ...")
        self._plan_client.wait_for_service(timeout_sec=10.0)
        self._logger.info("Waiting for /execute_trajectory ...")
        self._exec_client.wait_for_server(timeout_sec=10.0)
        self._logger.info("MoveIt2Client ready")

    # ── 位姿目标 ────────────────────────────────────────────────────────
    def plan_pose(self, x, y, z, qw=1.0, qx=0.0, qy=0.0, qz=0.0):
        """绝对笛卡尔位姿目标 → move_group 内部自动 IK → 规划 → 返回轨迹"""

        pos = PositionConstraint()
        pos.header.frame_id = BASE_LINK
        pos.link_name = END_EFFECTOR
        pos.weight = 1.0
        pos.constraint_region.primitives.append(
            SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.001])
        )
        pos.constraint_region.primitive_poses[0].position.x = x
        pos.constraint_region.primitive_poses[0].position.y = y
        pos.constraint_region.primitive_poses[0].position.z = z

        ori = OrientationConstraint()
        ori.header.frame_id = BASE_LINK
        ori.link_name = END_EFFECTOR
        ori.weight = 1.0
        ori.orientation.w = qw
        ori.orientation.x = qx
        ori.orientation.y = qy
        ori.orientation.z = qz
        ori.absolute_x_axis_tolerance = 0.01
        ori.absolute_y_axis_tolerance = 0.01
        ori.absolute_z_axis_tolerance = 0.01

        constraints = Constraints(
            position_constraints=[pos],
            orientation_constraints=[ori],
        )
        return self._plan(constraints)

    # ── 末端相对旋转 ────────────────────────────────────────────────────
    def plan_relative_pose(self, msg: PoseStamped):
        """
        在当前末端位姿上叠加相对变换，然后规划。

        当 msg.header.frame_id == END_EFFECTOR 时：
          - 从 TF 读取 L_base_link 在 base 下的当前位姿
          - 将 msg 中的相对位姿叠加到当前位姿上
          - 以叠加后的绝对位姿进行规划
        """
        if msg.header.frame_id == END_EFFECTOR:
            # 查 TF: L_base_link 在 base 下的当前位姿
            try:
                tf: TransformStamped = self._tf_buffer.lookup_transform(
                    BASE_LINK, END_EFFECTOR, rclpy.time.Time()
                )
            except Exception as e:
                self._logger.error(f"TF lookup failed: {e}")
                return None

            # 在当前末端位上叠加相对变换
            relative = PoseStamped()
            relative.header.frame_id = END_EFFECTOR
            relative.pose = msg.pose
            absolute = do_transform_pose(relative, tf)

            target_x = absolute.pose.position.x
            target_y = absolute.pose.position.y
            target_z = absolute.pose.position.z
            target_qw = absolute.pose.orientation.w
            target_qx = absolute.pose.orientation.x
            target_qy = absolute.pose.orientation.y
            target_qz = absolute.pose.orientation.z

            self._logger.info(
                f"Relative → Absolute: pos=({target_x:.3f},{target_y:.3f},{target_z:.3f})"
            )
        else:
            # 绝对位姿，直接用
            target_x = msg.pose.position.x
            target_y = msg.pose.position.y
            target_z = msg.pose.position.z
            target_qw = msg.pose.orientation.w
            target_qx = msg.pose.orientation.x
            target_qy = msg.pose.orientation.y
            target_qz = msg.pose.orientation.z

        return self.plan_pose(
            target_x, target_y, target_z,
            target_qw, target_qx, target_qy, target_qz,
        )

    # ── 关节目标 ────────────────────────────────────────────────────────
    def plan_joint(self, joint_positions):
        """关节角度目标 → 直接规划（跳过 IK）"""

        constraints = Constraints()
        for name, val in zip(JOINT_NAMES, joint_positions):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = val
            jc.tolerance_above = 0.001
            jc.tolerance_below = 0.001
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        return self._plan(constraints)

    # ── 内部：发送规划请求 ──────────────────────────────────────────────
    def _plan(self, goal_constraints):
        req = MotionPlanRequest()
        req.group_name = GROUP_NAME
        req.planner_id = PLANNER_ID
        req.allowed_planning_time = PLANNING_TIME
        req.num_planning_attempts = PLANNING_ATTEMPTS
        req.max_velocity_scaling_factor = VELOCITY_SCALE
        req.max_acceleration_scaling_factor = ACCELERATION_SCALE
        req.goal_constraints = [goal_constraints]
        req.start_state.is_diff = True  # 从当前关节状态开始

        srv_req = GetMotionPlan.Request()
        srv_req.motion_plan_request = req

        future = self._plan_client.call_async(srv_req)
        rclpy.spin_until_future_complete(self._node, future)

        if future.result() is None:
            self._logger.error("Plan service call failed")
            return None

        resp = future.result().motion_plan_response
        if resp.error_code.val != 1:  # 1 = SUCCESS
            self._logger.error(f"Plan failed: error_code={resp.error_code.val}")
            return None

        traj = resp.trajectory
        pts = len(traj.joint_trajectory.points)
        self._logger.info(f"Plan success: {pts} waypoints")
        return traj

    # ── 执行轨迹 ────────────────────────────────────────────────────────
    def execute(self, trajectory) -> bool:
        """通过 ExecuteTrajectory action 执行（阻塞）"""

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory

        send_future = self._exec_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self._node, send_future)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self._logger.error("Execute rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self._node, result_future)

        result = result_future.result().result
        if result.error_code.val == 1:
            self._logger.info("Execute done")
            return True
        else:
            self._logger.error(f"Execute failed: error_code={result.error_code.val}")
            return False


# =============================================================================
# ROS 2 节点
# =============================================================================
class PoseGoalNode(Node):

    def __init__(self):
        super().__init__("pose_goal_node")
        self._client = MoveIt2Client(self)

        # 绝对位姿控制话题：/pose_goal_node/target_pose
        self._abs_sub = self.create_subscription(
            PoseStamped, "~/target_pose", self._absolute_callback, 10
        )
        # 相对变换控制话题：/pose_goal_node/delta_pose
        self._rel_sub = self.create_subscription(
            PoseStamped, "~/delta_pose", self._relative_callback, 10
        )
        self.get_logger().info(
            "Ready.\n"
            "  绝对位姿: /pose_goal_node/target_pose\n"
            "  相对变换: /pose_goal_node/delta_pose"
        )

    def _absolute_callback(self, msg: PoseStamped):
        """绝对位姿目标：frame_id 为参考坐标系，目标为绝对位姿"""
        p = msg.pose.position
        o = msg.pose.orientation
        self.get_logger().info(
            f"[ABS  | {msg.header.frame_id}] "
            f"x={p.x:.3f} y={p.y:.3f} z={p.z:.3f} "
            f"qx={o.x:.3f} qy={o.y:.3f} qz={o.z:.3f} qw={o.w:.3f}"
        )
        traj = self._client.plan_pose(
            x=p.x, y=p.y, z=p.z,
            qw=o.w, qx=o.x, qy=o.y, qz=o.z,
        )
        if traj:
            self._client.execute(traj)

    def _relative_callback(self, msg: PoseStamped):
        """相对变换目标：在末端当前位姿上叠加 msg 中的变换"""
        p = msg.pose.position
        o = msg.pose.orientation
        self.get_logger().info(
            f"[REL  | {msg.header.frame_id}] "
            f"dx={p.x:.3f} dy={p.y:.3f} dz={p.z:.3f} "
            f"qx={o.x:.3f} qy={o.y:.3f} qz={o.z:.3f} qw={o.w:.3f}"
        )
        traj = self._client.plan_relative_pose(msg)
        if traj:
            self._client.execute(traj)


def main():
    rclpy.init()
    rclpy.spin(PoseGoalNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

### 3.2 注册入口点

编辑 `src/tianyi2_control/setup.py`（**唯一需要手动编辑的文件**）：

```python
entry_points={
    "console_scripts": [
        "pose_goal_node = tianyi2_control.pose_goal_node:main",
    ],
},
```

---

## 4. 编译

```bash
cd ~/moveit2_demo_ws
colcon build --packages-select tianyi2_control
source install/setup.bash
```

---

## 5. 运行与测试

### 5.1 启 demo
```bash
ros2 launch demo demo.launch.py
```

### 5.2 启控制节点
```bash
ros2 run tianyi2_control pose_goal_node
```

### 5.3 发位姿命令

```bash
# ── 绝对位姿：直接移动到指定坐标 ──
ros2 topic pub /pose_goal_node/target_pose geometry_msgs/msg/PoseStamped "
header:
  frame_id: base
pose:
  position: {x: 0.25, y: 0.1, z: 0.4}
  orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}
" -1

# ── 相对变换：末端原地 roll -90°（绕自身 X 轴旋转）──
ros2 topic pub /pose_goal_node/delta_pose geometry_msgs/msg/PoseStamped "
header:
  frame_id: base
pose:
  position: {x: 0.0, y: 0.0, z: 0.0}
  orientation: {w: 0.707, x: -0.707, y: 0.0, z: 0.0}
" -1
```

---

## 6. 后续添加新节点

加 `bridge_node.py` 到 `tianyi2_control/` 目录，在 `setup.py` 的 `console_scripts` 加一行：

```python
"bridge_node = tianyi2_control.bridge_node:main",
```

`package.xml` 不需要修改——所有依赖已在创建时声明好。

---

## 7. 架构总结

```
/pose_goal_node/target_pose (绝对位姿)
/pose_goal_node/delta_pose  (相对变换)
        │
        ▼
  PoseGoalNode._absolute_callback / _relative_callback
        │
        ├── MoveIt2Client.plan_pose() ─────────── 绝对
        │     └── /plan_kinematic_path (GetMotionPlan service)
        │
        ├── MoveIt2Client.plan_relative_pose() ── 相对
        │     ├── 查 TF: L_base_link 当前位姿
        │     ├── do_transform_pose() 叠加变换
        │     └── /plan_kinematic_path (GetMotionPlan service)
        │
        └── MoveIt2Client.execute(trajectory)
              └── /execute_trajectory (ExecuteTrajectory action)
                    move_group → FollowJointTrajectory → 控制器 → 硬件
```

依赖链：`rclpy` + `moveit_msgs` + `geometry_msgs` + `trajectory_msgs`，全部 apt 安装。
