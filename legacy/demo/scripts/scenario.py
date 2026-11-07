#!/usr/bin/env python3
"""Planar, deterministic module demo; publishes only under /scan_demo by default."""
import math
import struct
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import TransformBroadcaster


class Scenario(Node):
    def __init__(self):
        super().__init__("scan_scenario")
        self.declare_parameter("use_pcd_map", False)
        self.declare_parameter("waypoints", [0.0, 0.0, 0.6, 2.0, 0.0, 0.6])
        self.x = self.y = self.yaw = 0.0
        self.z = 0.6
        self.cmd = Twist()
        self.cmd_received = 0.0
        self.last = self.get_clock().now()
        self.last_map = 0.0
        self.sent_path = False
        self.points = None
        self.broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.cloud_pub = self.create_publisher(PointCloud2, "local_map", 10)
        durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, "global_path", durable)
        self.create_subscription(Twist, "cmd_vel", self.on_command, 10)
        self.create_subscription(PointCloud2, "map_source", self.on_map, durable)
        self.create_timer(0.02, self.tick)

    def on_command(self, message):
        self.cmd = message
        self.cmd_received = time.monotonic()

    def on_map(self, message):
        if message.header.frame_id != "scan_demo_odom":
            self.get_logger().error("Demo map_source must be in scan_demo_odom")
            return
        try:
            values = []
            for axis in "xyz":
                field = next(f for f in message.fields if f.name == axis)
                if field.datatype != PointField.FLOAT32 or field.count != 1:
                    raise ValueError("Expected FLOAT32 map coordinates")
                values.append(np.ndarray(
                    (message.height, message.width), dtype=">f4" if message.is_bigendian else "<f4",
                    buffer=message.data, offset=field.offset,
                    strides=(message.row_step, message.point_step)).reshape(-1))
            points = np.column_stack(values)
            self.points = points[np.isfinite(points).all(axis=1)].copy()
        except (ValueError, StopIteration) as error:
            self.get_logger().error(str(error))

    def publish_path(self, stamp):
        points = self.get_parameter("waypoints").value
        if len(points) < 3 or len(points) % 3:
            raise ValueError("waypoints must contain XYZ triples")
        path = Path(); path.header.frame_id = "scan_demo_map"; path.header.stamp = stamp
        for i in range(0, len(points), 3):
            pose = PoseStamped(); pose.header = path.header
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = points[i:i+3]
            pose.pose.orientation.w = 1.0; path.poses.append(pose)
        self.path_pub.publish(path); self.sent_path = True

    def tick(self):
        now = self.get_clock().now(); stamp = now.to_msg()
        dt = max(0.0, min(0.1, (now - self.last).nanoseconds / 1e9)); self.last = now
        cmd = self.cmd if time.monotonic() - self.cmd_received < 0.3 else Twist()
        vx = max(-0.75, min(0.75, cmd.linear.x)); vy = max(-0.35, min(0.35, cmd.linear.y))
        wz = max(-1.0, min(1.0, cmd.angular.z))
        self.x += (math.cos(self.yaw)*vx - math.sin(self.yaw)*vy)*dt
        self.y += (math.sin(self.yaw)*vx + math.cos(self.yaw)*vy)*dt
        self.yaw += wz*dt
        odom = Odometry(); odom.header.frame_id = "scan_demo_odom"; odom.header.stamp = stamp
        odom.child_frame_id = "scan_demo_base"
        odom.pose.pose.position.x = self.x; odom.pose.pose.position.y = self.y; odom.pose.pose.position.z = self.z
        odom.pose.pose.orientation.z = math.sin(self.yaw/2); odom.pose.pose.orientation.w = math.cos(self.yaw/2)
        odom.twist.twist.linear.x = vx; odom.twist.twist.linear.y = vy; odom.twist.twist.angular.z = wz
        self.odom_pub.publish(odom)
        root = TransformStamped(); root.header.frame_id = "scan_demo_map"; root.header.stamp = stamp
        root.child_frame_id = "scan_demo_odom"; root.transform.rotation.w = 1.0
        body = TransformStamped(); body.header = odom.header; body.child_frame_id = odom.child_frame_id
        body.transform.translation.x = self.x; body.transform.translation.y = self.y; body.transform.translation.z = self.z
        body.transform.rotation = odom.pose.pose.orientation
        self.broadcaster.sendTransform([root, body])
        wall = time.monotonic()
        if wall - self.last_map >= 0.5:
            if self.get_parameter("use_pcd_map").value:
                if self.points is None:
                    return
                points = self.points[np.all(np.abs(self.points - [self.x, self.y, self.z]) < [4.5, 4.5, 2.0], axis=1)]
            else:
                points = np.array([(x/5, y/5, 0.0) for x in range(-10, 26)
                                   for y in range(-10, 11)], dtype=np.float32)
            msg = PointCloud2(); msg.header.frame_id = "scan_demo_odom"; msg.header.stamp = stamp
            msg.height = 1; msg.width = len(points); msg.point_step = 12; msg.row_step = msg.width*12
            msg.fields = [PointField(name=axis, offset=i*4, datatype=PointField.FLOAT32, count=1)
                          for i, axis in enumerate("xyz")]
            msg.data = np.asarray(points, dtype="<f4").tobytes()
            self.cloud_pub.publish(msg); self.last_map = wall
        if not self.sent_path and self.path_pub.get_subscription_count():
            self.publish_path(stamp)


def main():
    rclpy.init()
    node = Scenario()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node(); rclpy.try_shutdown()


if __name__ == "__main__":
    main()
