#!/usr/bin/env python3
"""
天轶2.0 桥接节点 v5 — 双臂独立队列，差分速度

数据流:
  /L_Arm_Group_controller/state → 左臂队列 (FIFO)
  /R_Arm_Group_controller/state → 右臂队列 (FIFO)
  /arm/status                   → 电机实际位置

spd 和 tor 写死为 0，kp/kd 从配置文件逐电机读取。
"""

import os
import threading

import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
from bodyctrl_msgs.msg import (
    CmdMotorCtrl,
    MotorCtrl,
    MotorStatusMsg,
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
                    j2m = {}
                    for k, v in data.get("joint_to_motor", {}).items():
                        j2m[str(k)] = int(v)
                    kp_map = {}
                    for k, v in data.get("motor_kp", {}).items():
                        kp_map[int(k)] = float(v)
                    kd_map = {}
                    for k, v in data.get("motor_kd", {}).items():
                        kd_map[int(k)] = float(v)
                    kp_scale = float(data.get("kp_scale", 1.0))
                    return j2m, kp_map, kd_map, kp_scale
            except Exception:
                pass
    return {}, {}, {}, 1.0


JOINT_TO_MOTOR, MOTOR_KP, MOTOR_KD, KP_SCALE = _load_motor_config()

CONTROL_HZ = 50.0
REACHED_TOLERANCE = 0.05  # rad


class Arm:
    """单臂队列状态。queue: [{motor_id: pos}] """

    def __init__(self, side: str):
        self.side = side
        self.queue: list = []                       # [dict[int,float]]
        self.sent: dict[int, float] = {}            # 当前目标


class BridgeNode(Node):

    def __init__(self):
        super().__init__("bridge_node")

        self._l_arm = Arm("L")
        self._r_arm = Arm("R")

        self._motor_pos: dict[int, float] = {}
        self._lock = threading.Lock()

        # ── 调试计数 ──
        self._ready = False  # 收到首次 /arm/status 后才开始处理队列
        self._l_recv = 0
        self._r_recv = 0
        self._pub_count = 0
        self._missing_logged: set[int] = set()
        self._l_baseline: dict[int, float] = {}  # 左臂空闲基线
        self._r_baseline: dict[int, float] = {}  # 右臂空闲基线

        self._pub = self.create_publisher(CmdMotorCtrl, "/arm/cmd_ctrl", 10)

        self._create_ctrl_sub("/L_Arm_Group_controller/state", self._l_arm)
        self._create_ctrl_sub("/R_Arm_Group_controller/state", self._r_arm)
        self._status_sub = self.create_subscription(
            MotorStatusMsg, "/arm/status", self._on_motor_status, 10
        )
        self.get_logger().info("  Subscribed: /arm/status")

        period = 1.0 / CONTROL_HZ
        self._timer = self.create_timer(period, self._tick)

        if JOINT_TO_MOTOR:
            self.get_logger().info(
                f"Loaded config: {len(JOINT_TO_MOTOR)} joints → "
                f"{len(set(JOINT_TO_MOTOR.values()))} motors"
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
        cnt = [0]  # mutable counter for closure

        def callback(msg: JointTrajectoryControllerState):
            if not msg.joint_names:
                return

            wp: dict[int, float] = {}
            for name, pos in zip(msg.joint_names, msg.desired.positions):
                mid = JOINT_TO_MOTOR.get(name)
                if mid is not None:
                    wp[mid] = float(pos)

            if not wp:
                return

            with self._lock:
                baseline = self._l_baseline if arm.side == "L" else self._r_baseline
                if not baseline:
                    # 首帧控制器状态作为空闲基线，不下发
                    if arm.side == "L":
                        self._l_baseline = wp
                    else:
                        self._r_baseline = wp
                    self.get_logger().info(
                        f"[{arm.side}] Idle baseline captured ({len(wp)} motors)"
                    )
                    return
                # 丢弃等于空闲基线的路点
                if wp == baseline:
                    return
                if arm.queue and wp == arm.queue[-1]:
                    return
                arm.queue.append(wp)
                is_first = len(arm.queue) == 1

            cnt[0] += 1
            if is_first:
                self.get_logger().info(
                    f"[{arm.side}] Trajectory started ({len(wp)} motors)"
                )
            elif cnt[0] % 50 == 0:
                self.get_logger().debug(
                    f"[{arm.side}] Recv {cnt[0]}, queue={len(arm.queue)}"
                )

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
            if not self._ready:
                # 等待首次 /arm/status 到达
                has_any = any(
                    mid in self._motor_pos for mid in JOINT_TO_MOTOR.values()
                )
                if not has_any:
                    return
                # 首次: 清空空闲路点，以当前电机位置作为初始目标
                self._l_arm.queue.clear()
                self._r_arm.queue.clear()
                init = {}
                for mid in JOINT_TO_MOTOR.values():
                    cur = self._motor_pos.get(mid)
                    if cur is not None:
                        init[mid] = cur
                if init:
                    self._l_arm.sent = {mid: init[mid] for mid in init if mid <= 17}
                    self._r_arm.sent = {mid: init[mid] for mid in init if mid >= 21}
                self._ready = True
                self.get_logger().info(
                    f"Initialized from /arm/status — holding {len(init)} motors"
                )
                return

            pos = dict(self._motor_pos)

            for arm in (self._l_arm, self._r_arm):
                if not arm.queue:
                    continue
                wp = arm.queue[0]

                if not arm.sent:
                    arm.sent = wp
                    arm.queue.pop(0)
                    changed = True
                    vals = ", ".join(f"{mid}={wp[mid]:.3f}" for mid in sorted(wp.keys()))
                    self.get_logger().info(
                        f"[{arm.side}] INITIAL SEND: {vals}"
                    )

                elif self._reached(arm.sent, pos):
                    arm.sent = wp
                    arm.queue.pop(0)
                    changed = True
                    self.get_logger().info(
                        f"[{arm.side}] REACHED → NEXT: queue={len(arm.queue)}"
                    )

            if changed:
                self._publish_combined()

    # =========================================================================
    # 到达判断
    # =========================================================================
    def _reached(self, targets: dict[int, float], current: dict[int, float]) -> bool:
        for mid, tgt in targets.items():
            cur = current.get(mid)
            if cur is None:
                if mid not in self._missing_logged:
                    self.get_logger().warn(f"  Motor {mid} not in /arm/status")
                    self._missing_logged.add(mid)
                return False
            if abs(cur - tgt) > REACHED_TOLERANCE:
                return False
        return True

    # =========================================================================
    # 合并双臂 → /arm/cmd_pos
    # =========================================================================
    def _publish_combined(self):
        cmd = CmdMotorCtrl()
        cmd.header.stamp = self.get_clock().now().to_msg()

        for mid in sorted(JOINT_TO_MOTOR.values()):
            arm = self._l_arm if mid <= 17 else self._r_arm
            pos = arm.sent.get(mid)
            if pos is None:
                continue
            mc = MotorCtrl()
            mc.name = mid
            mc.kp = MOTOR_KP.get(mid, 0.0) * KP_SCALE
            mc.kd = MOTOR_KD.get(mid, 0.0)
            mc.pos = pos
            mc.spd = 0.0
            mc.tor = 0.0
            cmd.cmds.append(mc)

        self._pub.publish(cmd)

        self._pub_count += 1
        if self._pub_count <= 3 or self._pub_count % 50 == 0:
            self.get_logger().info(
                f"[PUB #{self._pub_count}] {len(cmd.cmds)} motors → /arm/cmd_ctrl"
            )


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
