#!/usr/bin/env python3
"""Publish a configured PCD-aligned reference path without simulating sensors or odometry."""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class PathPublisher(Node):
    def __init__(self):
        super().__init__("scenario_path")
        values = self.declare_parameter("waypoints", Parameter.Type.DOUBLE_ARRAY).value
        self.frame_id = self.declare_parameter("frame_id", "scan_demo_map").value
        if len(values) < 6 or len(values) % 3 or not all(math.isfinite(float(v)) for v in values):
            raise ValueError("waypoints must contain at least two finite XYZ triples")
        self.values = values
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(Path, "global_path", qos)
        self.timer = self.create_timer(0.2, self.publish_once)

    def publish_once(self):
        if not self.publisher.get_subscription_count():
            return
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.frame_id
        for i in range(0, len(self.values), 3):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.orientation.w = 1.0
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = self.values[i:i + 3]
            path.poses.append(pose)
        self.publisher.publish(path)
        self.get_logger().info(f"Published {len(path.poses)} configured XYZ poses")
        self.timer.cancel()


def main():
    rclpy.init()
    node = None
    try:
        node = PathPublisher()
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
