#!/usr/bin/env python3
"""
天轶2.0 位姿目标控制节点（双规划组）
直调 move_group 的 /plan_kinematic_path service + /execute_trajectory action

话题：
  /left_arm/target_pose   — 左臂绝对位姿 (base 系)
  /left_arm/delta_pose    — 左臂相对变换 (平移 base 系 | 旋转 base 系*末端朝向)
  /right_arm/target_pose  — 右臂绝对位姿 (base 系)
  /right_arm/delta_pose   — 右臂相对变换 (平移 base 系 | 旋转 base 系*末端朝向)
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
# 天轶2.0 双臂参数
# =============================================================================
BASE_LINK = "base"

ARMS = {
    "left_arm": {
        "group": "L_Arm_Group",
        "effector": "L_base_link",
        "joints": [
            "shoulder_roll_l_joint", "shoulder_yaw_l_joint",
            "elbow_pitch_l_joint",   "elbow_yaw_l_joint",
            "wrist_pitch_l_joint",   "wrist_roll_l_joint",
        ],
    },
    "right_arm": {
        "group": "R_Arm_group",
        "effector": "R_base_link",
        "joints": [
            "shoulder_roll_r_joint", "shoulder_yaw_r_joint",
            "elbow_pitch_r_joint",   "elbow_yaw_r_joint",
            "wrist_pitch_r_joint",   "wrist_roll_r_joint",
        ],
    },
}

PLANNER_ID = "RRTConnectkConfigDefault"
PLANNING_TIME = 5.0
PLANNING_ATTEMPTS = 10
VELOCITY_SCALE = 0.5
ACCELERATION_SCALE = 0.5


class MoveIt2Client:
    """MoveIt 2 轻量客户端，绑定一个规划组。"""

    def __init__(self, node: Node, group: str, effector: str, joints: list):
        self._node = node
        self._logger = node.get_logger()
        self._group = group
        self._effector = effector
        self._joints = joints

        self._plan_client = node.create_client(
            GetMotionPlan, "/plan_kinematic_path"
        )
        self._exec_client = ActionClient(
            node, ExecuteTrajectory, "/execute_trajectory"
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, node)

        self._logger.info(f"[{group}] Waiting for /plan_kinematic_path ...")
        self._plan_client.wait_for_service(timeout_sec=10.0)
        self._logger.info(f"[{group}] Waiting for /execute_trajectory ...")
        self._exec_client.wait_for_server(timeout_sec=10.0)
        self._logger.info(f"[{group}] MoveIt2Client ready")

    # =========================================================================
    # 绝对位姿目标（base 系）
    # =========================================================================
    def plan_pose(self, x, y, z, qw=1.0, qx=0.0, qy=0.0, qz=0.0):
        constraints = Constraints()
        constraints.position_constraints.append(
            self._make_position_constraint(x, y, z)
        )
        constraints.orientation_constraints.append(
            self._make_orientation_constraint(qw, qx, qy, qz)
        )
        return self._plan(constraints)

    # =========================================================================
    # 相对变换：平移 base 系 + 旋转 base 系*末端朝向
    # =========================================================================
    def plan_relative_pose(self, dx, dy, dz, d_qw, d_qx, d_qy, d_qz):
        try:
            tf: TransformStamped = self._tf_buffer.lookup_transform(
                BASE_LINK, self._effector, rclpy.time.Time()
            )
        except Exception as e:
            self._logger.error(f"[{self._group}] TF lookup failed: {e}")
            return None

        # 当前位置 + base 系平移
        cx = tf.transform.translation.x + dx
        cy = tf.transform.translation.y + dy
        cz = tf.transform.translation.z + dz

        # 当前朝向
        c_qx = tf.transform.rotation.x
        c_qy = tf.transform.rotation.y
        c_qz = tf.transform.rotation.z
        c_qw = tf.transform.rotation.w

        # 旋转：末端自身系 = current * delta
        r_qw = c_qw*d_qw - c_qx*d_qx - c_qy*d_qy - c_qz*d_qz
        r_qx = c_qw*d_qx + c_qx*d_qw + c_qy*d_qz - c_qz*d_qy
        r_qy = c_qw*d_qy - c_qx*d_qz + c_qy*d_qw + c_qz*d_qx
        r_qz = c_qw*d_qz + c_qx*d_qy - c_qy*d_qx + c_qz*d_qw

        self._logger.info(
            f"[{self._group}] Delta → Abs: "
            f"pos=({cx:.3f},{cy:.3f},{cz:.3f}) "
            f"qw={r_qw:.3f} qx={r_qx:.3f} qy={r_qy:.3f} qz={r_qz:.3f}"
        )
        return self.plan_pose(x=cx, y=cy, z=cz,
                              qw=r_qw, qx=r_qx, qy=r_qy, qz=r_qz)

    # =========================================================================
    # 关节目标
    # =========================================================================
    def plan_joint(self, joint_positions):
        constraints = Constraints()
        for name, val in zip(self._joints, joint_positions):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = val
            jc.tolerance_above = 0.001
            jc.tolerance_below = 0.001
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        return self._plan(constraints)

    # =========================================================================
    # 内部：发送规划
    # =========================================================================
    def _plan(self, goal_constraints):
        req = MotionPlanRequest()
        req.group_name = self._group
        req.planner_id = PLANNER_ID
        req.allowed_planning_time = PLANNING_TIME
        req.num_planning_attempts = PLANNING_ATTEMPTS
        req.max_velocity_scaling_factor = VELOCITY_SCALE
        req.max_acceleration_scaling_factor = ACCELERATION_SCALE
        req.goal_constraints = [goal_constraints]
        req.start_state.is_diff = True

        srv_req = GetMotionPlan.Request()
        srv_req.motion_plan_request = req

        self._logger.info(f"[{self._group}] Calling /plan_kinematic_path ...")
        future = self._plan_client.call_async(srv_req)
        rclpy.spin_until_future_complete(
            self._node, future, timeout_sec=PLANNING_TIME + 3.0
        )

        if future.result() is None:
            self._logger.error(f"[{self._group}] Plan service call timed out")
            return None

        resp = future.result().motion_plan_response
        if resp.error_code.val != 1:
            self._logger.error(
                f"[{self._group}] Plan failed: error_code={resp.error_code.val}"
            )
            return None

        traj = resp.trajectory
        pts = len(traj.joint_trajectory.points)
        last_pt = traj.joint_trajectory.points[-1]
        self._logger.info(
            f"[{self._group}] Plan OK: {pts} waypoints, "
            f"duration={last_pt.time_from_start.sec:.2f}s"
        )
        return traj

    # =========================================================================
    # 执行
    # =========================================================================
    def execute(self, trajectory):
        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = trajectory

        self._logger.info(f"[{self._group}] Sending execute goal...")
        send_future = self._exec_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self._node, send_future, timeout_sec=5.0)

        if send_future.result() is None:
            self._logger.error(f"[{self._group}] Execute send timed out")
            return False

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self._logger.error(f"[{self._group}] Execute rejected")
            return False

        self._logger.info(f"[{self._group}] Executing...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self._node, result_future, timeout_sec=30.0)

        result = result_future.result()
        if result is None:
            self._logger.error(f"[{self._group}] Execute result timed out")
            return False

        if result.result.error_code.val == 1:
            self._logger.info(f"[{self._group}] Execute done")
            return True
        else:
            self._logger.error(
                f"[{self._group}] Execute failed: "
                f"code={result.result.error_code.val}"
            )
            return False

    # =========================================================================
    # 约束 helper
    # =========================================================================
    def _make_position_constraint(self, x, y, z):
        pc = PositionConstraint()
        pc.header.frame_id = BASE_LINK
        pc.link_name = self._effector
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
        oc.link_name = self._effector
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

        # 左臂客户端
        l = ARMS["left_arm"]
        self._left = MoveIt2Client(self, l["group"], l["effector"], l["joints"])
        # 右臂客户端
        r = ARMS["right_arm"]
        self._right = MoveIt2Client(self, r["group"], r["effector"], r["joints"])

        # left_arm 话题
        self._l_abs = self.create_subscription(
            PoseStamped, "left_arm/target_pose",
            lambda m: self._abs_callback(m, self._left), 10,
        )
        self._l_rel = self.create_subscription(
            PoseStamped, "left_arm/delta_pose",
            lambda m: self._rel_callback(m, self._left), 10,
        )
        # right_arm 话题
        self._r_abs = self.create_subscription(
            PoseStamped, "right_arm/target_pose",
            lambda m: self._abs_callback(m, self._right), 10,
        )
        self._r_rel = self.create_subscription(
            PoseStamped, "right_arm/delta_pose",
            lambda m: self._rel_callback(m, self._right), 10,
        )

        self.get_logger().info(
            "Ready.\n"
            "  左臂 ABS : /pose_goal_node/left_arm/target_pose\n"
            "  左臂 REL : /pose_goal_node/left_arm/delta_pose\n"
            "  右臂 ABS : /pose_goal_node/right_arm/target_pose\n"
            "  右臂 REL : /pose_goal_node/right_arm/delta_pose"
        )

    def _abs_callback(self, msg: PoseStamped, arm: MoveIt2Client):
        p = msg.pose.position
        o = msg.pose.orientation
        arm._logger.info(
            f"[ABS] x={p.x:.3f} y={p.y:.3f} z={p.z:.3f}"
        )
        threading.Thread(
            target=self._do_plan_and_execute,
            args=(arm, p.x, p.y, p.z, o.w, o.x, o.y, o.z),
            daemon=True,
        ).start()

    def _rel_callback(self, msg: PoseStamped, arm: MoveIt2Client):
        p = msg.pose.position
        o = msg.pose.orientation
        arm._logger.info(
            f"[REL] dx={p.x:.3f} dy={p.y:.3f} dz={p.z:.3f} "
            f"qx={o.x:.3f} qy={o.y:.3f} qz={o.z:.3f} qw={o.w:.3f}"
        )
        threading.Thread(
            target=self._do_rel_plan_and_execute,
            args=(arm, p.x, p.y, p.z, o.w, o.x, o.y, o.z),
            daemon=True,
        ).start()

    def _do_plan_and_execute(self, arm, x, y, z, qw, qx, qy, qz):
        traj = arm.plan_pose(x=x, y=y, z=z, qw=qw, qx=qx, qy=qy, qz=qz)
        if traj:
            arm.execute(traj)

    def _do_rel_plan_and_execute(self, arm, dx, dy, dz, d_qw, d_qx, d_qy, d_qz):
        traj = arm.plan_relative_pose(
            dx=dx, dy=dy, dz=dz,
            d_qw=d_qw, d_qx=d_qx, d_qy=d_qy, d_qz=d_qz,
        )
        if traj:
            arm.execute(traj)


def main():
    rclpy.init()
    node = PoseGoalNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
