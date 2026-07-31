#!/usr/bin/env python3
"""读取左右臂末端位姿，输出可直接调用的控制指令。"""

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

FRAME_TO_TOPIC = {
    "L_base_link": "/left_arm/target_pose",
    "R_base_link": "/right_arm/target_pose",
}


class EEPoseReader(Node):

    def __init__(self):
        super().__init__("ee_pose_reader")
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._timer = self.create_timer(0.5, self._lookup)

    def _lookup(self):
        for frame, topic in FRAME_TO_TOPIC.items():
            try:
                t = self._tf_buffer.lookup_transform(
                    "base", frame, rclpy.time.Time()
                )
                p = t.transform.translation
                r = t.transform.rotation
                print(
                    f'ros2 topic pub {topic} geometry_msgs/msg/PoseStamped '
                    f'"pose: {{position: {{x: {p.x:.4f}, y: {p.y:.4f}, z: {p.z:.4f}}}, '
                    f'orientation: {{x: {r.x:.4f}, y: {r.y:.4f}, z: {r.z:.4f}, w: {r.w:.4f}}}}}" -1'
                )
            except Exception as e:
                self.get_logger().warn(f"Waiting for {frame}: {e}")

        self.destroy_node()
        rclpy.shutdown()


def main():
    rclpy.init()
    node = EEPoseReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
