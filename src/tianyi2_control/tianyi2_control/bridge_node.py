#!/usr/bin/env python3
"""
天轶2.0 桥接节点 v2
订阅 move_group 的 /display_planned_path → 逐路点闭环执行 → /arm/cmd_pos

- 接收新轨迹时替换当前待执行队列
- 异步线程 10Hz：读取 /arm/status → 判断是否到达当前路点（误差 < 0.05 rad）
- 到达后发送下一路点，直到轨迹执行完毕
"""

import os
import threading
import time

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import DisplayTrajectory
from bodyctrl_msgs.msg import CmdSetMotorPosition, SetMotorPosition, MotorStatusMsg

# ── YAML 解析 ──
try:
    import yaml
except ImportError:
    yaml = None

# ── 查找包共享目录 ──
try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None


# =============================================================================
# 加载电机配置 (motor_config.yaml)
# =============================================================================
def _load_motor_config() -> tuple[dict[str, int], dict[int, float]]:
    """加载 motor_config.yaml，返回 (joint_to_motor, motor_currents)。"""
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
                    raw_j2m = data.get("joint_to_motor", {})
                    if isinstance(raw_j2m, dict):
                        for k, v in raw_j2m.items():
                            j2m[str(k)] = int(v)

                    currents = {}
                    raw_cur = data.get("motor_currents", {})
                    if isinstance(raw_cur, dict):
                        for k, v in raw_cur.items():
                            currents[int(k)] = float(v)

                    return j2m, currents
            except Exception:
                pass
    return {}, {}

JOINT_TO_MOTOR, MOTOR_CURRENTS = _load_motor_config()

# 到达判断容差 (rad)
REACHED_TOLERANCE = 0.05
# 控制循环频率 (Hz)
CONTROL_HZ = 10


class BridgeNode(Node):

    def __init__(self):
        super().__init__("bridge_node")

        # ── 发布：电机位置指令 ──
        self._pub = self.create_publisher(
            CmdSetMotorPosition, "/arm/cmd_pos", 10
        )

        # ── 订阅：move_group 规划轨迹 ──
        self._traj_sub = self.create_subscription(
            DisplayTrajectory, "/display_planned_path",
            self._on_trajectory, 10,
        )

        # ── 订阅：机器人当前关节状态 ──
        self._status_sub = self.create_subscription(
            MotorStatusMsg, "/arm/status",
            self._on_status, 10,
        )

        # ── 轨迹缓存 ──
        self._waypoints: list[tuple[int, ...]] = []    # [(motor_id, pos), ...] per point
        self._motor_ids: list[int] = []
        self._current_wp_index = 0
        self._side = "?"

        # ── 状态缓存 ──
        self._current_pos: dict[int, float] = {}       # motor_id → pos (rad)
        self._lock = threading.Lock()

        # ── 控制线程 ──
        self._ctrl_thread = threading.Thread(target=self._control_loop, daemon=True)
        self._ctrl_thread.start()

        loaded = len(MOTOR_CURRENTS)
        if loaded > 0:
            self.get_logger().info(
                f"Loaded motor currents for {loaded} motors: "
                f"{dict(list(MOTOR_CURRENTS.items())[:3])}..."
            )
        else:
            self.get_logger().warn(
                "No motor_config.yaml found — using cur=0.0 for all motors"
            )

        self.get_logger().info(
            "Bridge ready: /display_planned_path → /arm/cmd_pos "
            "(closed-loop, 10Hz, ±0.05rad)"
        )

    # =========================================================================
    # 轨迹回调：替换待执行队列
    # =========================================================================
    def _on_trajectory(self, msg: DisplayTrajectory):
        if not msg.trajectory:
            return

        jt = msg.trajectory[0].joint_trajectory
        if not jt.points or not jt.joint_names:
            return

        # 过滤出已映射的关节
        indices = []
        motor_ids = []
        for i, name in enumerate(jt.joint_names):
            mid = JOINT_TO_MOTOR.get(name)
            if mid is not None:
                indices.append(i)
                motor_ids.append(mid)

        if not motor_ids:
            return

        side = "L" if "_l_" in jt.joint_names[0] else "R"
        waypoints = []
        for point in jt.points:
            waypoints.append(
                tuple(float(point.positions[idx]) for idx in indices)
            )

        with self._lock:
            self._waypoints = waypoints
            self._motor_ids = motor_ids
            self._current_wp_index = 0
            self._side = side

        self.get_logger().info(
            f"[{self._side}] New trajectory: {len(waypoints)} pts, "
            f"{len(motor_ids)} motors"
        )

    # =========================================================================
    # 状态回调
    # =========================================================================
    def _on_status(self, msg: MotorStatusMsg):
        with self._lock:
            for s in msg.status:
                self._current_pos[s.name] = float(s.pos)

    # =========================================================================
    # 控制循环 (10 Hz)
    # =========================================================================
    def _control_loop(self):
        rate = self.create_rate(CONTROL_HZ)
        while rclpy.ok():
            self._tick()
            rate.sleep()

    def _tick(self):
        with self._lock:
            wps = self._waypoints
            mids = self._motor_ids
            idx = self._current_wp_index
            current_pos = dict(self._current_pos)

        # 无待执行轨迹
        if not wps or idx >= len(wps):
            return

        target = wps[idx]

        # 检查当前路点是否已到达
        if self._is_reached(target, mids, current_pos):
            with self._lock:
                if self._current_wp_index == idx:  # 未被新轨迹覆盖
                    self._current_wp_index += 1

            idx += 1
            if idx >= len(wps):
                self.get_logger().info(
                    f"[{self._side}] Trajectory complete "
                    f"({len(wps)} pts)"
                )
                return

            target = wps[idx]

        # 发送当前路点
        self._publish_waypoint(target, mids)

    # =========================================================================
    # 到达判断
    # =========================================================================
    def _is_reached(self, target, motor_ids, current_pos):
        for motor_id, tgt in zip(motor_ids, target):
            cur = current_pos.get(motor_id)
            if cur is None:
                return False  # 还没收到该关节状态
            if abs(cur - tgt) > REACHED_TOLERANCE:
                return False
        return True

    # =========================================================================
    # 发送单路点
    # =========================================================================
    def _publish_waypoint(self, target, motor_ids):
        cmd = CmdSetMotorPosition()
        cmd.header.stamp = self.get_clock().now().to_msg()

        for motor_id, pos in zip(motor_ids, target):
            sp = SetMotorPosition()
            sp.name = motor_id
            sp.pos = pos
            sp.spd = 0.0
            sp.cur = MOTOR_CURRENTS.get(motor_id, 0.0)
            cmd.cmds.append(sp)

        self._pub.publish(cmd)


def main():
    rclpy.init()
    node = BridgeNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
