"""Isolated synthetic integration; all velocity output stays under /scan_it."""
import math
import struct
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from scan_planner_msgs.msg import PlannerStatus


@pytest.mark.launch_test
def generate_test_description():
    common = {
        "global_frame": "scan_it_map", "planning_frame": "scan_it_odom",
        "robot_frame": "scan_it_reference", "control_frame": "scan_it_reference",
        "unknown_space_policy": "free_inside_bounds", "velocity_source": "pose_difference",
        "odom_timeout": 0.4, "map_timeout": 0.7,
    }
    planner = launch_ros.actions.Node(
        package="scan_planner", executable="planner_node", namespace="scan_it",
        parameters=[common, {"replan_period": 0.2, "trajectory_lease": 0.6}], output="screen")
    controller = launch_ros.actions.Node(
        package="scan_planner", executable="controller_node", namespace="scan_it",
        parameters=[common], output="screen")
    # A single loop wake lets launch_testing deliver shutdown after fixture I/O stops.
    wake = launch.actions.TimerAction(period=20.0, actions=[])
    return launch.LaunchDescription([planner, controller, wake, launch_testing.actions.ReadyToTest()])


class TestNavigationContracts(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_navigation_and_failure_contracts(self):
        node = rclpy.create_node("scan_contract_fixture")
        try:
            broadcaster = TransformBroadcaster(node)
            static_broadcaster = StaticTransformBroadcaster(node)
            installation = TransformStamped()
            installation.header.stamp = node.get_clock().now().to_msg()
            installation.header.frame_id = "scan_it_base"
            installation.child_frame_id = "scan_it_reference"
            installation.transform.translation.x = 0.1
            installation.transform.rotation.w = 1.0
            static_broadcaster.sendTransform(installation)
            odom_pub = node.create_publisher(Odometry, "/scan_it/odom", 10)
            cloud_pub = node.create_publisher(PointCloud2, "/scan_it/local_map", 10)
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
            path_pub = node.create_publisher(Path, "/scan_it/global_path", qos)
            statuses, commands, local_paths = [], [], []
            node.create_subscription(PlannerStatus, "/scan_it/scan_planner/status",
                                     lambda msg: statuses.append(msg), qos)
            node.create_subscription(Twist, "/scan_it/cmd_vel",
                                     lambda msg: commands.append(msg), 10)
            node.create_subscription(Path, "/scan_it/scan_planner/local_path",
                                     lambda msg: local_paths.append(msg), qos)
            position = [0.0, 0.0, 0.6]
            settings = {"odom": True, "cloud": True, "frozen_stamp": None,
                        "map_shift": 0.0, "blocked": False, "wrong_tf_orientation": False}

            def to_map(x, y, z):
                return (1.0 + math.cos(0.4)*x - math.sin(0.4)*y,
                        2.0 + math.sin(0.4)*x + math.cos(0.4)*y, z + 0.1)

            def publish_path(empty=False, frame="scan_it_map"):
                msg = Path()
                msg.header.frame_id = frame
                msg.header.stamp = node.get_clock().now().to_msg()
                if not empty:
                    for point in [(0.0, 0.0, 0.6), (1.0, 0.0, 0.6), (2.0, 0.0, 0.6)]:
                        pose = PoseStamped(); pose.header = msg.header
                        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = to_map(*point)
                        pose.pose.orientation.w = 1.0
                        msg.poses.append(pose)
                path_pub.publish(msg)

            def feed():
                stamp = node.get_clock().now().to_msg()
                transform = TransformStamped(); transform.header.stamp = stamp
                transform.header.frame_id = "scan_it_map"; transform.child_frame_id = "scan_it_odom"
                transform.transform.translation.x = 1.0 + settings["map_shift"]
                transform.transform.translation.y = 2.0; transform.transform.translation.z = 0.1
                transform.transform.rotation.z = math.sin(0.2); transform.transform.rotation.w = math.cos(0.2)
                transforms = [transform]
                if settings["odom"]:
                    odom = Odometry(); odom.header.stamp = stamp
                    odom.header.frame_id = "scan_it_odom"; odom.child_frame_id = "scan_it_base"
                    odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = position
                    odom.pose.pose.orientation.w = 1.0
                    # Twist intentionally remains zero, as in the inspected Fast-LIO.
                    odom_pub.publish(odom)
                    body = TransformStamped(); body.header = odom.header
                    body.child_frame_id = "scan_it_base"
                    body.transform.translation.x, body.transform.translation.y, body.transform.translation.z = position
                    angle = 0.8 if settings["wrong_tf_orientation"] else 0.0
                    body.transform.rotation.z = math.sin(angle/2)
                    body.transform.rotation.w = math.cos(angle/2); transforms.append(body)
                broadcaster.sendTransform(transforms)
                if settings["cloud"]:
                    msg = PointCloud2(); msg.header.frame_id = "scan_it_odom"
                    msg.header.stamp = settings["frozen_stamp"] or stamp
                    msg.height = 1; msg.width = 1; msg.point_step = 12; msg.row_step = 12
                    msg.fields = [PointField(name=axis, offset=i*4, datatype=PointField.FLOAT32, count=1)
                                  for i, axis in enumerate("xyz")]
                    msg.data = struct.pack("fff", 0.1, 0.0, 0.6) if settings["blocked"] else struct.pack("fff", 3.0, 3.0, 0.0)
                    cloud_pub.publish(msg)

            timer = node.create_timer(0.05, feed)

            def wait_for(predicate, timeout=12.0):
                end = time.monotonic() + timeout
                while time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=0.02)
                    if predicate():
                        return
                self.fail("Timed out; last states: %s" % [(s.state, s.reason) for s in statuses[-5:]])

            def pump(duration):
                end = time.monotonic() + duration
                while time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=0.02)

            def moving():
                return any(abs(m.linear.x) + abs(m.linear.y) > 0.001 for m in commands[-10:])

            def stopped():
                return len(commands) >= 10 and all(
                    abs(m.linear.x)+abs(m.linear.y)+abs(m.angular.z) < 1e-8 for m in commands[-10:])

            wait_for(lambda: path_pub.get_subscription_count() > 0 and len(statuses) > 0)
            publish_path()
            wait_for(moving)
            valid = [p for p in local_paths if p.poses]
            self.assertTrue(valid)
            self.assertEqual(valid[-1].header.frame_id, "scan_it_odom")
            self.assertAlmostEqual(valid[-1].poses[0].pose.position.x, 0.1, delta=0.03)
            self.assertAlmostEqual(valid[-1].poses[0].pose.position.z, 0.6, delta=0.03)

            # A new occupied snapshot invalidates an already active trajectory.
            settings["blocked"] = True
            wait_for(lambda: statuses[-1].state == PlannerStatus.LOCAL_PLAN_FAILED and stopped())
            pump(0.3); self.assertTrue(stopped())
            settings["blocked"] = False
            wait_for(moving)

            # Translation agrees within tolerance, but orientation does not.
            settings["wrong_tf_orientation"] = True
            wait_for(lambda: statuses[-1].state == PlannerStatus.LOCALIZATION_UNAVAILABLE and stopped())
            settings["wrong_tf_orientation"] = False
            wait_for(moving)

            # PCT failure semantics: empty path revokes the running task.
            publish_path(empty=True)
            wait_for(lambda: statuses[-1].state == PlannerStatus.GLOBAL_PATH_INVALID and stopped())
            pump(0.4); self.assertTrue(stopped())

            # A wrong global frame must never reuse the old path.
            publish_path(frame="wrong_map")
            wait_for(lambda: statuses[-1].state == PlannerStatus.GLOBAL_PATH_INVALID and stopped())

            publish_path(); wait_for(moving)
            settings["frozen_stamp"] = node.get_clock().now().to_msg()
            pump(1.0)
            wait_for(stopped)
            self.assertFalse(statuses[-1].motion_allowed)
            settings["frozen_stamp"] = None
            wait_for(moving)

            # Large applied map->odom correction invalidates old trajectories.
            start = len(statuses); settings["map_shift"] = 0.5
            wait_for(lambda: any(s.state == PlannerStatus.TF_CHANGED for s in statuses[start:]))
            settings["map_shift"] = 0.0
            pump(0.3); publish_path(); wait_for(moving)

            settings["odom"] = False
            pump(0.8); wait_for(stopped)
            settings["odom"] = True; pump(0.3)

            # Same XY on the wrong floor is not goal reached.
            position[:] = [2.0, 0.0, 1.6]
            pump(0.8)
            self.assertNotEqual(statuses[-1].state, PlannerStatus.GOAL_REACHED)
            position[:] = [2.0, 0.0, 0.6]
            wait_for(lambda: statuses[-1].state == PlannerStatus.GOAL_REACHED and stopped())
            position[:] = [0.0, 0.0, 0.6]
            pump(0.6)
            self.assertEqual(statuses[-1].state, PlannerStatus.GOAL_REACHED)
            self.assertTrue(stopped())
            node.destroy_timer(timer)
        finally:
            node.destroy_node()
