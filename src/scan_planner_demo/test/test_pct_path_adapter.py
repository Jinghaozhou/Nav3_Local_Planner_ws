"""Check live PCT Path conversion and task revocation."""

import time
import unittest

import launch
import launch_testing.actions
import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from launch.actions import TimerAction
from launch_ros.actions import Node
from nav_msgs.msg import Path
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


@pytest.mark.launch_test
def generate_test_description():
    adapter = Node(package="scan_planner_demo", executable="pct_path_adapter.py",
                   namespace="pct_it", output="screen", parameters=[{
                       "input_topic": "/pct_it/pct_path", "source_frame": "map",
                       "target_frame": "scan_demo_map", "z_offset": -0.10,
                   }])
    return launch.LaunchDescription([adapter, launch_testing.actions.ReadyToTest(),
                                     TimerAction(period=20.0, actions=[])])


class TestPctPathAdapter(unittest.TestCase):
    def test_path_height_frame_and_empty_revoke(self):
        rclpy.init()
        node = rclpy.create_node("pct_path_adapter_probe")
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        publisher = node.create_publisher(Path, "/pct_it/pct_path", qos)
        received = []
        node.create_subscription(Path, "/pct_it/global_path", received.append, qos)

        def send_until(message, predicate):
            deadline = time.monotonic() + 10.0
            while time.monotonic() < deadline:
                publisher.publish(message)
                rclpy.spin_once(node, timeout_sec=0.1)
                if any(predicate(item) for item in received):
                    return next(item for item in reversed(received) if predicate(item))
            self.fail("PCT path adapter did not publish the expected Path")

        try:
            path = Path()
            path.header.frame_id = "map"
            for x in (3.0, 4.0):
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.pose.position.x = x
                pose.pose.position.z = 0.60
                pose.pose.orientation.w = 1.0
                path.poses.append(pose)
            adapted = send_until(path, lambda m: len(m.poses) == 2)
            self.assertEqual(adapted.header.frame_id, "scan_demo_map")
            self.assertEqual(adapted.poses[0].header.frame_id, "scan_demo_map")
            self.assertAlmostEqual(adapted.poses[0].pose.position.z, 0.50)
            self.assertAlmostEqual(adapted.poses[1].pose.position.x, 4.0)
            empty = Path()
            empty.header.frame_id = "map"
            revoked = send_until(empty, lambda m: len(m.poses) == 0)
            self.assertEqual(revoked.header.frame_id, "scan_demo_map")
        finally:
            node.destroy_node()
            rclpy.shutdown()
