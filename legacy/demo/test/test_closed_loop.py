"""Verify that the shipped demo closes the velocity/odometry loop and stops."""
import time
import unittest
from pathlib import Path

import launch
import launch_testing.actions
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scan_planner_msgs.msg import PlannerStatus


@pytest.mark.launch_test
def generate_test_description():
    share = Path(get_package_share_directory("scan_planner_demo"))
    demo = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        str(share / "launch" / "simulation.launch.py")))
    return launch.LaunchDescription([demo, launch_testing.actions.ReadyToTest()])


class TestClosedLoop(unittest.TestCase):
    def test_reaches_planar_goal_and_publishes_zero_velocity(self):
        rclpy.init()
        node = rclpy.create_node("closed_loop_observer")
        states, odometry, commands = [], [], []
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(PlannerStatus, "/scan_demo/scan_planner/status",
                                 lambda msg: states.append(msg), qos)
        node.create_subscription(Odometry, "/scan_demo/odom",
                                 lambda msg: odometry.append(msg), 10)
        node.create_subscription(Twist, "/scan_demo/cmd_vel",
                                 lambda msg: commands.append(msg), 10)
        try:
            deadline = time.monotonic() + 65.0
            reached = False
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                if states and states[-1].state == PlannerStatus.GOAL_REACHED:
                    reached = True
                    break
            self.assertTrue(reached, "Demo did not reach goal; last states=%s, x=%s" % (
                [(s.state, s.reason) for s in states[-4:]],
                odometry[-1].pose.pose.position.x if odometry else None))
            end = time.monotonic() + 0.4
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.02)
            self.assertTrue(any(abs(c.linear.x) > 0.001 for c in commands))
            self.assertTrue(odometry)
            self.assertGreater(odometry[-1].pose.pose.position.x, 1.8)
            self.assertLess(odometry[-1].pose.pose.position.x, 2.2)
            self.assertAlmostEqual(odometry[-1].pose.pose.position.z, 0.6, delta=1e-6)
            self.assertTrue(len(commands) >= 10)
            self.assertTrue(all(abs(c.linear.x)+abs(c.linear.y)+abs(c.angular.z)<1e-8 for c in commands[-10:]))
        finally:
            node.destroy_node()
            rclpy.shutdown()
