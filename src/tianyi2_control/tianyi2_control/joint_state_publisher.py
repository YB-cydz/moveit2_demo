#!/usr/bin/env python3
"""
天轶2.0 关节状态发布器
从真实硬件状态话题 → /joint_states

订阅:
  /arm/status            (MotorStatusMsg)   手臂 14 电机
  /head/status           (MotorStatusMsg)   头部 3 电机
  /waist/status          (MotorStatusMsg)   腰部电机
  /leg/status            (MotorStatusMsg)   腿部 2 电机

发布:
  /joint_states          (sensor_msgs/JointState)  全机器人关节状态

手指关节不通过 MoveIt 控制，不出现在 /joint_states 和 TF 中。
"""

import os
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from bodyctrl_msgs.msg import MotorStatusMsg

try:
    import yaml
except ImportError:
    yaml = None

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None


# =============================================================================
# 加载配置
# =============================================================================
def _load_config() -> dict:
    """加载 joint_state_config.yaml，返回配置字典。"""
    paths = []
    if get_package_share_directory is not None:
        try:
            share = get_package_share_directory("tianyi2_control")
            paths.append(os.path.join(share, "config", "joint_state_config.yaml"))
        except Exception:
            pass
    paths.append(
        os.path.join(os.path.dirname(__file__), "..", "config", "joint_state_config.yaml")
    )

    for p in paths:
        if os.path.isfile(p) and yaml is not None:
            try:
                with open(p, "r", encoding="utf-8") as f:
                    data = yaml.safe_load(f)
                if isinstance(data, dict):
                    return data
            except Exception:
                pass

    return {}


Config = _load_config()

# 电机 ID (int) → 关节名 (str), 按话题分组
ARM_MAP   = {int(k): str(v) for k, v in Config.get("arm_motor_to_joint", {}).items()}
HEAD_MAP  = {int(k): str(v) for k, v in Config.get("head_motor_to_joint", {}).items()}
WAIST_MAP = {int(k): str(v) for k, v in Config.get("waist_motor_to_joint", {}).items()}
LEG_MAP   = {int(k): str(v) for k, v in Config.get("leg_motor_to_joint", {}).items()}

# 手部: 字符串 name (如 '1') → URDF 关节名
LEFT_HAND_MAP  = {str(k): str(v) for k, v in Config.get("left_hand_to_joint", {}).items()}
RIGHT_HAND_MAP = {str(k): str(v) for k, v in Config.get("right_hand_to_joint", {}).items()}

# 固定关节 (无传感器)
FIXED_JOINTS = [str(j) for j in Config.get("fixed_joints", [])]

# 标定偏移 (硬件值 + offset = 发布值)
JOINT_OFFSETS = {str(k): float(v) for k, v in Config.get("joint_offsets", {}).items()}

PUBLISH_RATE = float(Config.get("publish_rate", 50.0))
FRAME_ID = str(Config.get("frame_id", "base"))


class JointStatePublisher(Node):
    """聚合所有硬件状态话题，发布完整 /joint_states。"""

    def __init__(self):
        super().__init__("joint_state_publisher")

        # ── 关节位置缓存 (joint_name → position_rad) ──
        self._positions: dict[str, float] = {}
        self._lock = threading.Lock()

        # 固定关节初始化为 0（无传感器，始终不变）
        for j in FIXED_JOINTS:
            self._positions[j] = 0.0

        # ── 发布者: /joint_states ──
        self._pub = self.create_publisher(JointState, "/joint_states", 10)

        # ── 定时发布 ──
        period = 1.0 / max(PUBLISH_RATE, 1.0)
        self._timer = self.create_timer(period, self._publish)

        # ── 订阅: MotorStatusMsg 话题 ──
        self._create_motor_sub("/arm/status", ARM_MAP)
        self._create_motor_sub("/head/status", HEAD_MAP)
        self._create_motor_sub("/waist/status", WAIST_MAP)
        self._create_motor_sub("/leg/status", LEG_MAP)

        # ── 订阅: 手部 JointState ──
        self._create_hand_sub("/inspire_hand/state/left_hand", LEFT_HAND_MAP)
        self._create_hand_sub("/inspire_hand/state/right_hand", RIGHT_HAND_MAP)

        # ── 统计日志 ──
        mapped = (len(ARM_MAP) + len(HEAD_MAP) + len(WAIST_MAP) + len(LEG_MAP)
                  + len(LEFT_HAND_MAP) + len(RIGHT_HAND_MAP))
        self.get_logger().info(
            f"JointStatePublisher ready @ {PUBLISH_RATE:.0f}Hz "
            f"({mapped} mapped joints, {len(FIXED_JOINTS)} fixed)"
        )

    # =========================================================================
    # 订阅工厂
    # =========================================================================
    def _create_motor_sub(self, topic: str, motor_map: dict[int, str]):
        """创建 MotorStatusMsg 订阅，绑定 motor_id → joint_name 映射。"""
        if not motor_map:
            self.get_logger().debug(f"Skipping {topic}: empty motor map")
            return  # 不订阅空映射的话题

        def callback(msg: MotorStatusMsg):
            with self._lock:
                for s in msg.status:
                    jname = motor_map.get(int(s.name))
                    if jname:
                        self._positions[jname] = float(s.pos)

        self.create_subscription(MotorStatusMsg, topic, callback, 10)
        self.get_logger().info(f"  Subscribed: {topic} ({len(motor_map)} motors)")

    def _create_hand_sub(self, topic: str, hand_map: dict[str, str]):
        """创建手部 JointState 订阅，重映射 name → URDF 关节名。"""
        if not hand_map:
            self.get_logger().debug(f"Skipping {topic}: empty hand map")
            return

        def callback(msg: JointState):
            with self._lock:
                for i, name in enumerate(msg.name):
                    jname = hand_map.get(str(name))
                    if jname and i < len(msg.position):
                        self._positions[jname] = float(msg.position[i])

        self.create_subscription(JointState, topic, callback, 10)
        self.get_logger().info(f"  Subscribed: {topic} ({len(hand_map)} fingers)")

    # =========================================================================
    # 发布 /joint_states
    # =========================================================================
    def _publish(self):
        with self._lock:
            joint_names = list(self._positions.keys())
            positions = [
                self._positions[n] + JOINT_OFFSETS.get(n, 0.0)
                for n in joint_names
            ]

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = FRAME_ID
        msg.name = joint_names
        msg.position = positions
        # 速度/力矩不发布（硬件状态中有 speed/current，但 joint_states 一般只填 position）
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = JointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
