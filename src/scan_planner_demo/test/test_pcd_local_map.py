"""Exercise PCD loading, odometry-based crop, frame, and ROS publication."""

import time
import unittest
from pathlib import Path

import launch
import launch_testing.actions
import pytest
import rclpy
from launch.actions import TimerAction
from launch_ros.actions import Node
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


@pytest.mark.launch_test
def generate_test_description():
    fixture = Path(__file__).parent / "fixtures" / "cloud.pcd"
    process = Node(
        package="scan_planner_demo", executable="pcd_local_map_node", namespace="pcd_it",
        parameters=[{"pcd_file": str(fixture), "frame_id": "scan_demo_odom",
                     "downsample_resolution": 0.1, "crop_half_xy": 1.0,
                     "crop_half_z": 1.0, "publish_rate": 10.0,
                     "world_bounds": [-1.0, 1.0, -1.0, 1.0, -0.1, 0.3]}],
        output="screen")
    # Wake launch_testing's event loop when assertions finish in restricted DDS setups.
    return launch.LaunchDescription([process, launch_testing.actions.ReadyToTest(),
                                     TimerAction(period=20.0, actions=[])])


class TestPcdLocalMap(unittest.TestCase):
    def test_crops_using_odometry(self):
        rclpy.init()
        node = rclpy.create_node("pcd_local_map_probe")
        publisher = node.create_publisher(Odometry, "/pcd_it/odom", qos_profile_sensor_data)
        received = []
        node.create_subscription(PointCloud2, "/pcd_it/local_map",
                                 lambda message: received.append(message), qos_profile_sensor_data)
        try:
            deadline = time.monotonic() + 10.0
            while time.monotonic() < deadline and not received:
                odom = Odometry()
                odom.header.frame_id = "scan_demo_odom"
                odom.header.stamp = node.get_clock().now().to_msg()
                odom.child_frame_id = "base"
                odom.pose.pose.orientation.w = 1.0
                publisher.publish(odom)
                rclpy.spin_once(node, timeout_sec=0.1)
            self.assertTrue(received, "PCD node did not publish after odometry")
            self.assertEqual(received[0].header.frame_id, "scan_demo_odom")
            self.assertEqual(received[0].width, 3)
            self.assertTrue({field.name for field in received[0].fields} >= {"x", "y", "z"})
        finally:
            node.destroy_node()
            rclpy.shutdown()
