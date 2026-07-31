#!/usr/bin/env python3
"""
天轶2.0 桥接节点 v5 — 双臂独立队列，差分速度

数据流:
  /L_Arm_Group_controller/state → 左臂队列 (FIFO)
  /R_Arm_Group_controller/state → 右臂队列 (FIFO)
  /arm/status                   → 电机实际位置

速度:
  spd (rad/s) = (路点pos - 上一路点pos) / dt × speed_ratio
  超过 motor_speed_limits[RPM] 的截断到限速值（0 = 不限速）
"""

import math
import os
import threading

import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
from bodyctrl_msgs.msg import (
    CmdSetMotorPosition,
    MotorStatusMsg,
    SetMotorPosition,
)

try:
    import yaml
except ImportError:
    yaml = None

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None


# =============================================================================
# 加载电机配置
# =============================================================================
def _load_motor_config():
    paths = []
    if get_package_share_directory is not None:
        try:
            share = get_package_share_directory("tianyi2_control")
            paths.append(os.path.join(share, "config", "motor_config.yaml"))
        except Exception:
            pass
    paths.append(
        os.path.join(os.path.dirname(__file__), "..", "config", "motor_config.yaml")
    )

    for p in paths:
        if os.path.isfile(p) and yaml is not None:
            try:
                with open(p, "r", encoding="utf-8") as f:
                    data = yaml.safe_load(f)
                if isinstance(data, dict):
                    j2m, cur, lim = {}, {}, {}
                    for k, v in data.get("joint_to_motor", {}).items():
                        j2m[str(k)] = int(v)
                    for k, v in data.get("motor_currents", {}).items():
                        cur[int(k)] = float(v)
                    for k, v in data.get("motor_speed_limits", {}).items():
                        lim[int(k)] = float(v)
                    ratio = float(data.get("speed_ratio", 0.5))
                    return j2m, cur, lim, ratio
            except Exception:
                pass
    return {}, {}, {}, 0.5


JOINT_TO_MOTOR, MOTOR_CURRENTS, MOTOR_SPEED_LIMITS, SPEED_RATIO = _load_motor_config()

CONTROL_HZ = 50.0
REACHED_TOLERANCE = 0.05  # rad
RPM_TO_RAD_S = 2.0 * math.pi / 60.0


class Arm:
    """单臂队列状态。queue: [(timestamp_sec, {motor_id: pos})] """

    def __init__(self, side: str):
        self.side = side
        self.queue: list = []                       # [(float, dict[int,float])]
        self.sent: dict[int, float] = {}            # 当前目标
        self.speeds: dict[int, float] = {}          # 计算出的 spd (rad/s)
        self.sent_time: float = 0.0                 # 当前目标的时间戳


class BridgeNode(Node):

    def __init__(self):
        super().__init__("bridge_node")

        self._l_arm = Arm("L")
        self._r_arm = Arm("R")

        self._motor_pos: dict[int, float] = {}
        self._lock = threading.Lock()

        self._pub = self.create_publisher(CmdSetMotorPosition, "/arm/cmd_pos", 10)

        self._create_ctrl_sub("/L_Arm_Group_controller/state", self._l_arm)
        self._create_ctrl_sub("/R_Arm_Group_controller/state", self._r_arm)
        self._status_sub = self.create_subscription(
            MotorStatusMsg, "/arm/status", self._on_motor_status, 10
        )
        self.get_logger().info("  Subscribed: /arm/status")

        period = 1.0 / CONTROL_HZ
        self._timer = self.create_timer(period, self._tick)

        loaded = len(MOTOR_CURRENTS)
        if loaded > 0:
            self.get_logger().info(
                f"Loaded config: {len(JOINT_TO_MOTOR)} joints → "
                f"{loaded} motors, {len(MOTOR_SPEED_LIMITS)} speed limits, "
                f"speed_ratio={SPEED_RATIO}"
            )
        else:
            self.get_logger().warn("No motor_config.yaml found")

        self.get_logger().info(
            f"Bridge ready: L+R queues, {CONTROL_HZ:.0f}Hz, "
            f"tolerance=±{REACHED_TOLERANCE}rad"
        )

    # =========================================================================
    # 控制器状态 → 入队 (时间戳 + 位置)
    # =========================================================================
    def _create_ctrl_sub(self, topic: str, arm: Arm):
        def callback(msg: JointTrajectoryControllerState):
            if not msg.joint_names:
                return

            now = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9

            wp: dict[int, float] = {}
            for name, pos in zip(msg.joint_names, msg.desired.positions):
                mid = JOINT_TO_MOTOR.get(name)
                if mid is not None:
                    wp[mid] = float(pos)

            if not wp:
                return

            with self._lock:
                if arm.queue:
                    _, last_wp = arm.queue[-1]
                    if wp == last_wp:
                        return
                arm.queue.append((now, wp))

        self.create_subscription(
            JointTrajectoryControllerState, topic, callback, 10
        )
        self.get_logger().info(f"  Subscribed: {topic}")

    # =========================================================================
    # 电机实际状态
    # =========================================================================
    def _on_motor_status(self, msg: MotorStatusMsg):
        with self._lock:
            for s in msg.status:
                self._motor_pos[int(s.name)] = float(s.pos)

    # =========================================================================
    # 主循环
    # =========================================================================
    def _tick(self):
        changed = False

        with self._lock:
            pos = dict(self._motor_pos)

            for arm in (self._l_arm, self._r_arm):
                if not arm.queue:
                    continue
                t_now, wp = arm.queue[0]

                if not arm.sent:
                    arm.sent = wp
                    arm.sent_time = t_now
                    arm.speeds = {mid: 0.0 for mid in wp}
                    arm.queue.pop(0)
                    changed = True

                elif self._reached(arm.sent, pos):
                    dt = t_now - arm.sent_time
                    self._compute_speeds(arm, wp, dt)
                    arm.sent = wp
                    arm.sent_time = t_now
                    arm.queue.pop(0)
                    changed = True

            if changed:
                self._publish_combined()

    # =========================================================================
    # 速度: spd = (wp - sent) / dt × ratio, 截断到限速
    # =========================================================================
    def _compute_speeds(self, arm: Arm, wp: dict[int, float], dt: float):
        arm.speeds.clear()
        if dt < 1e-6:
            dt = 0.02

        for mid, new_pos in wp.items():
            old_pos = arm.sent.get(mid, new_pos)
            dp = new_pos - old_pos
            spd = (dp / dt) * SPEED_RATIO

            limit_rpm = MOTOR_SPEED_LIMITS.get(mid, 0.0)
            if limit_rpm > 0:
                limit_rad_s = limit_rpm * RPM_TO_RAD_S
                spd = max(-limit_rad_s, min(limit_rad_s, spd))

            arm.speeds[mid] = spd

    # =========================================================================
    # 到达判断
    # =========================================================================
    def _reached(self, targets: dict[int, float], current: dict[int, float]) -> bool:
        for mid, tgt in targets.items():
            cur = current.get(mid)
            if cur is None:
                return False
            if abs(cur - tgt) > REACHED_TOLERANCE:
                return False
        return True

    # =========================================================================
    # 合并双臂 → /arm/cmd_pos
    # =========================================================================
    def _publish_combined(self):
        cmd = CmdSetMotorPosition()
        cmd.header.stamp = self.get_clock().now().to_msg()

        for mid in sorted(JOINT_TO_MOTOR.values()):
            pos = self._l_arm.sent.get(mid) or self._r_arm.sent.get(mid)
            if pos is None:
                continue
            sp = SetMotorPosition()
            sp.name = mid
            sp.pos = pos
            sp.spd = self._l_arm.speeds.get(mid, self._r_arm.speeds.get(mid, 0.0))
            sp.cur = MOTOR_CURRENTS.get(mid, 0.0)
            cmd.cmds.append(sp)

        self._pub.publish(cmd)


def main():
    rclpy.init()
    node = BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
