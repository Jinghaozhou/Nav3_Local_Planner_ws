#!/usr/bin/env python3
"""Feed a live PCT Path into the PCD-aligned SCAN demo frame."""

import copy
import math

import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class PctPathAdapter(Node):
    def __init__(self):
        super().__init__("pct_path_adapter")
        input_topic = self.declare_parameter("input_topic", "/pct/global_path").value
        self.source_frame = self.declare_parameter("source_frame", "map").value
        self.target_frame = self.declare_parameter("target_frame", "scan_demo_map").value
        self.z_offset = float(self.declare_parameter("z_offset", -0.10).value)
        if (not input_topic.startswith("/") or input_topic == self.resolve_topic_name("global_path")
                or not self.source_frame or not self.target_frame or not math.isfinite(self.z_offset)):
            raise ValueError("Invalid PCT path adapter topic, frame or height offset")
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(Path, "global_path", qos)
        self.subscription = self.create_subscription(Path, input_topic, self.on_path, qos)
        self.get_logger().info(f"Waiting for PCT path on {input_topic}")

    def on_path(self, source):
        output = Path()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = self.target_frame
        valid = source.header.frame_id == self.source_frame
        if valid:
            for pose in source.poses:
                point = pose.pose.position
                if (pose.header.frame_id not in ("", self.source_frame)
                        or not all(math.isfinite(v) for v in (point.x, point.y, point.z))):
                    valid = False
                    break
                adapted = copy.deepcopy(pose)
                adapted.header = output.header
                adapted.pose.position.z += self.z_offset
                output.poses.append(adapted)
        if not valid:
            self.get_logger().error("Invalid PCT path frame or coordinates; clearing demo reference path")
            output.poses.clear()
        self.publisher.publish(output)
        self.get_logger().info(f"Forwarded {len(output.poses)} PCT poses to demo frame")


def main():
    rclpy.init()
    node = None
    try:
        node = PctPathAdapter()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
