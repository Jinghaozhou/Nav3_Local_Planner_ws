#!/usr/bin/env python3
"""Publish the documented five-point Mode 3 test path."""

import argparse
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.utilities import remove_ros_args


TARGET_WAYPOINTS = (
    (3.37, 0.0, 0.3),
    (2.87, -5.0, 0.3),
    (8.38, -5.0, 1.8),
    (2.87, -5.0, 3.3),
    (8.38, -5.0, 4.8),
)


class TestPathPublisher(Node):
    def __init__(self, topic, frame_id, planner_z_offset):
        super().__init__("scan_planner_test_path_publisher")
        self.topic = topic
        self.frame_id = frame_id
        self.planner_z_offset = planner_z_offset
        self.publisher = self.create_publisher(Path, topic, 10)

    def build_path(self):
        message = Path()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id

        for x, y, target_z in TARGET_WAYPOINTS:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            # Mode 3 adds grid_map.body_height to every received path point.
            pose.pose.position.z = target_z - self.planner_z_offset
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)

        return message

    def publish_once(self, timeout):
        deadline = time.monotonic() + max(0.0, timeout)
        while self.count_subscribers(self.topic) == 0 and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.count_subscribers(self.topic) == 0:
            self.get_logger().warning(
                f"No subscriber found on {self.topic}; publishing once anyway"
            )

        self.publisher.publish(self.build_path())
        self.get_logger().info(
            f"Published {len(TARGET_WAYPOINTS)} poses to {self.topic} in {self.frame_id}; "
            f"compensated planner z offset {self.planner_z_offset:.3f} m"
        )
        end_time = time.monotonic() + 0.5
        while time.monotonic() < end_time:
            rclpy.spin_once(self, timeout_sec=0.05)


def main():
    parser = argparse.ArgumentParser(
        description="Publish the five RUNNING.md waypoints as nav_msgs/msg/Path"
    )
    parser.add_argument("--topic", default="/initial_path")
    parser.add_argument("--frame-id", default="world")
    parser.add_argument(
        "--planner-z-offset",
        type=float,
        default=0.4,
        help="z offset Mode 3 adds internally; normally grid_map.body_height",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=10.0,
        help="seconds to wait for a matching subscriber",
    )
    arguments = parser.parse_args(remove_ros_args(args=sys.argv)[1:])

    rclpy.init(args=sys.argv)
    publisher = TestPathPublisher(
        arguments.topic, arguments.frame_id, arguments.planner_z_offset
    )
    try:
        publisher.publish_once(arguments.timeout)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
