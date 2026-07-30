#!/usr/bin/env python3
"""
天轶2.0 位姿目标控制节点
直调 move_group 的 /plan_kinematic_path service + /execute_trajectory action

全部写死 base 坐标系：
  /pose_goal_node/target_pose  — 绝对位姿（base 系）
  /pose_goal_node/delta_pose   — 相对旋转（base 系，位置保持不变）
"""

import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, Point, PoseStamped, TransformStamped
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    JointConstraint,
)
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.action import ExecuteTrajectory
from tf2_ros import Buffer, TransformListener


# =============================================================================
# 天轶2.0 左臂参数（来自 SRDF 和 URDF）
# =============================================================================
GROUP_NAME = "hand_L_Arn_Group"
BASE_LINK = "base"
END_EFFECTOR = "L_base_link"
JOINT_NAMES = [
    "shoulder_roll_l_joint",
    "shoulder_yaw_l_joint",
    "elbow_pitch_l_joint",
    "elbow_yaw_l_joint",
    "wrist_pitch_l_joint",
    "wrist_roll_l_joint",
]

PLANNER_ID = "RRTConnectkConfigDefault"
PLANNING_TIME = 5.0
PLANNING_ATTEMPTS = 10
VELOCITY_SCALE = 0.5
ACCELERATION_SCALE = 0.5


class MoveIt2Client:
    """MoveIt 2 轻量客户端，封装 move_group service/action。"""

    def __init__(self, node: Node):
        self._node = node
        self._logger = node.get_logger()

        self._plan_client = node.create_client(
            GetMotionPlan, "/plan_kinematic_path"
        )
        self._exec_client = ActionClient(
            node, ExecuteTrajectory, "/execute_trajectory"
        )

        # TF 监听（delta_pose 需要读取末端当前位姿）
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, node)

        self._logger.info("Waiting for /plan_kinematic_path ...")
        self._plan_client.wait_for_service(timeout_sec=10.0)
        self._logger.info("Waiting for /execute_trajectory ...")
        self._exec_client.wait_for_server(timeout_sec=10.0)
        self._logger.info("MoveIt2Client ready")

    # =========================================================================
    # 绝对位姿目标（base 系）
    # =========================================================================
    def plan_pose(self, x, y, z, qw=1.0, qx=0.0, qy=0.0, qz=0.0):
        """移动到 base 系下的绝对位姿。"""
        constraints = Constraints()
        constraints.position_constraints.append(
            self._make_position_constraint(x, y, z)
        )
        constraints.orientation_constraints.append(
            self._make_orientation_constraint(qw, qx, qy, qz)
        )
        return self._plan(constraints)

    # =========================================================================
    # 相对变换（全部 base 系）
    # =========================================================================
    def plan_relative_pose(self, d_qw, d_qx, d_qy, d_qz):
        """
        在末端当前位置上叠加 base 系旋转（不移位）。

        位置保持不变，旋转：delta * current
        """
        try:
            tf: TransformStamped = self._tf_buffer.lookup_transform(
                BASE_LINK, END_EFFECTOR, rclpy.time.Time()
            )
        except Exception as e:
            self._logger.error(f"TF lookup failed: {e}")
            return None

        # 当前位置（保持不变）
        cx, cy, cz = (
            tf.transform.translation.x,
            tf.transform.translation.y,
            tf.transform.translation.z,
        )

        # 当前朝向
        c_qx, c_qy, c_qz, c_qw = (
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z,
            tf.transform.rotation.w,
        )

        # 旋转：delta * current（base 系）
        r_qw = d_qw*c_qw - d_qx*c_qx - d_qy*c_qy - d_qz*c_qz
        r_qx = d_qw*c_qx + d_qx*c_qw + d_qy*c_qz - d_qz*c_qy
        r_qy = d_qw*c_qy - d_qx*c_qz + d_qy*c_qw + d_qz*c_qx
        r_qz = d_qw*c_qz + d_qx*c_qy - d_qy*c_qx + d_qz*c_qw

        self._logger.info(
            f"Delta rot → Abs: x={cx:.3f} y={cy:.3f} z={cz:.3f} "
            f"qw={r_qw:.3f} qx={r_qx:.3f} qy={r_qy:.3f} qz={r_qz:.3f}"
        )
        return self.plan_pose(
            x=cx, y=cy, z=cz, qw=r_qw, qx=r_qx, qy=r_qy, qz=r_qz,
        )

    # =========================================================================
    # 关节目标规划
    # =========================================================================
    def plan_joint(self, joint_positions):
        """规划到指定关节角度（跳过 IK）。"""
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

    # =========================================================================
    # 内部：发送规划请求
    # =========================================================================
    def _plan(self, goal_constraints):
        req = MotionPlanRequest()
        req.group_name = GROUP_NAME
        req.planner_id = PLANNER_ID
        req.allowed_planning_time = PLANNING_TIME
        req.num_planning_attempts = PLANNING_ATTEMPTS
        req.max_velocity_scaling_factor = VELOCITY_SCALE
        req.max_acceleration_scaling_factor = ACCELERATION_SCALE
        req.goal_constraints = [goal_constraints]
        req.start_state.is_diff = True

        srv_req = GetMotionPlan.Request()
        srv_req.motion_plan_request = req

        self._logger.info("Calling /plan_kinematic_path ...")
        future = self._plan_client.call_async(srv_req)
        rclpy.spin_until_future_complete(
            self._node, future, timeout_sec=PLANNING_TIME + 3.0
        )

        if future.result() is None:
            self._logger.error("Plan service call timed out")
            return None

        resp = future.result().motion_plan_response
        if resp.error_code.val != 1:
            self._logger.error(f"Plan failed: error_code={resp.error_code.val}")
            return None

        traj = resp.trajectory
        pts = len(traj.joint_trajectory.points)
        last_pt = traj.joint_trajectory.points[-1]
        self._logger.info(
            f"Plan OK: {pts} waypoints, "
            f"duration={last_pt.time_from_start.sec:.2f}s"
        )
        return traj

    # =========================================================================
    # 执行轨迹
    # =========================================================================
    def execute(self, trajectory):
        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = trajectory

        self._logger.info("Sending execute goal...")
        send_future = self._exec_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self._node, send_future, timeout_sec=5.0)

        if send_future.result() is None:
            self._logger.error("Execute send timed out")
            return False

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self._logger.error("Execute rejected")
            return False

        self._logger.info("Executing...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self._node, result_future, timeout_sec=30.0)

        result = result_future.result()
        if result is None:
            self._logger.error("Execute result timed out")
            return False

        if result.result.error_code.val == 1:
            self._logger.info("Execute done")
            return True
        else:
            self._logger.error(
                f"Execute failed: code={result.result.error_code.val}"
            )
            return False

    # =========================================================================
    # 构建约束消息 helper
    # =========================================================================
    def _make_position_constraint(self, x, y, z):
        pc = PositionConstraint()
        pc.header.frame_id = BASE_LINK
        pc.link_name = END_EFFECTOR
        pc.weight = 1.0
        sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.001])
        pc.constraint_region.primitives.append(sphere)
        pose = Pose()
        pose.position = Point(x=x, y=y, z=z)
        pc.constraint_region.primitive_poses.append(pose)
        return pc

    def _make_orientation_constraint(self, qw, qx, qy, qz):
        oc = OrientationConstraint()
        oc.header.frame_id = BASE_LINK
        oc.link_name = END_EFFECTOR
        oc.weight = 1.0
        oc.orientation.w = qw
        oc.orientation.x = qx
        oc.orientation.y = qy
        oc.orientation.z = qz
        oc.absolute_x_axis_tolerance = 0.01
        oc.absolute_y_axis_tolerance = 0.01
        oc.absolute_z_axis_tolerance = 0.01
        return oc


# =============================================================================
# ROS 2 节点
# =============================================================================
class PoseGoalNode(Node):

    def __init__(self):
        super().__init__("pose_goal_node")
        self._client = MoveIt2Client(self)

        # 绝对位姿：base 系
        self._abs_sub = self.create_subscription(
            PoseStamped, "~/target_pose", self._absolute_callback, 10
        )
        # 相对变换：平移 base 系 + 旋转 base 系
        self._rel_sub = self.create_subscription(
            PoseStamped, "~/delta_pose", self._relative_callback, 10
        )
        self.get_logger().info(
            "Ready.\n"
            "  [ABS]  /pose_goal_node/target_pose\n"
            "  [REL]  /pose_goal_node/delta_pose"
        )

    # ── 绝对位姿 ─────────────────────────────────────────────────────────
    def _absolute_callback(self, msg: PoseStamped):
        p = msg.pose.position
        o = msg.pose.orientation
        self.get_logger().info(
            f"[ABS] x={p.x:.3f} y={p.y:.3f} z={p.z:.3f}"
        )
        threading.Thread(
            target=self._do_plan_and_execute,
            args=(p.x, p.y, p.z, o.w, o.x, o.y, o.z),
            daemon=True,
        ).start()

    # ── 相对旋转 ─────────────────────────────────────────────────────────
    def _relative_callback(self, msg: PoseStamped):
        o = msg.pose.orientation
        self.get_logger().info(
            f"[REL] qx={o.x:.3f} qy={o.y:.3f} qz={o.z:.3f} qw={o.w:.3f}"
        )
        threading.Thread(
            target=self._do_relative_plan_and_execute,
            args=(o.w, o.x, o.y, o.z),
            daemon=True,
        ).start()

    # ── 规划 + 执行 ──────────────────────────────────────────────────────
    def _do_plan_and_execute(self, x, y, z, qw, qx, qy, qz):
        traj = self._client.plan_pose(x=x, y=y, z=z, qw=qw, qx=qx, qy=qy, qz=qz)
        if traj:
            self._client.execute(traj)

    def _do_relative_plan_and_execute(self, d_qw, d_qx, d_qy, d_qz):
        traj = self._client.plan_relative_pose(
            d_qw=d_qw, d_qx=d_qx, d_qy=d_qy, d_qz=d_qz,
        )
        if traj:
            self._client.execute(traj)


def main():
    rclpy.init()
    node = PoseGoalNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
