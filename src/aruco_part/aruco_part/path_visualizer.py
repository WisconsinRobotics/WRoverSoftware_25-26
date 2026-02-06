#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker


def yaw_to_quaternion(yaw):
    """Convert a yaw angle (rad) into a quaternion (z,w only, roll=pitch=0)."""
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class PathVisualizer(Node):
    def __init__(self):
        super().__init__('path_visualizer')

        # Sub to your fake_sensors / localization topic
        self.loc_sub = self.create_subscription(
            Float32MultiArray,
            'localization_info',       # <- change if your topic name is different
            self.localization_cb,
            10
        )
        self.imu_sub = self.create_subscription(
            Float32MultiArray,
            'imu_info',       # <- change if your topic name is different
            self.imu_cb,
            10
        )

        # Robot “icon”
        self.robot_pub = self.create_publisher(Marker, 'robot_marker', 10)
        # Trail / path
        self.path_pub = self.create_publisher(Path, 'robot_path', 10)
        self.plan_pub   = self.create_publisher(Marker, 'planned_path_markers', 10)

        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"  # frame RViz will use

        # Optional: limit number of poses stored
        self.max_points = 2000

        self.loc_msg = None
        self.imu_msg = None

        self.planned_published = False

    def publish_planned_path(self):
        """Publish the 5 semicircles as LINE_STRIP markers."""
        if self.planned_published:
            return
        self.planned_published = True

        radii = [20.0, 17.5, 15.0, 12.5, 10.0]
        angle_step = 0.02  # rad

        for idx, r in enumerate(radii):
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "planned_path"
            m.id = idx
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD

            # LINE_STRIP uses scale.x as line width
            m.scale.x = 0.1
            m.color.r = 1.0
            m.color.g = 1.0
            m.color.b = 1.0
            m.color.a = 0.7

            # even index → upper semicircle (0..π)
            # odd index → lower semicircle (π..2π)
            if idx % 2 == 0:
                start = 0.0
                end = math.pi
            else:
                start = math.pi
                end = 2.0 * math.pi

            theta = start
            while theta <= end + 1e-6:
                p = Point()
                p.x = r * math.cos(theta)
                p.y = r * math.sin(theta)
                p.z = 0.0
                m.points.append(p)
                theta += angle_step

            self.plan_pub.publish(m)
            # RViz will keep it; no need for lifetime

    def localization_cb(self, msg):
        self.loc_msg = msg
        self.callback()

    def imu_cb(self, msg):
        self.imu_msg = msg
        self.callback()

    def callback(self):
        if (self.loc_msg is None or
            self.imu_msg is None):
            return

        loc = self.loc_msg
        imu = self.imu_msg
        # Adjust this to whatever layout you use.
        # Here: [x, y, yaw]
        try:
            x = float(loc.data[0])
            y = float(loc.data[1])
            yaw = float(imu.data[3])
        except (IndexError, ValueError):
            self.get_logger().warn("Localization_info message has wrong format")
            return

        stamp = self.get_clock().now().to_msg()

        if not self.planned_published:
            self.publish_planned_path()

        # -----------------------
        # Robot marker (a flat cube)
        # -----------------------
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = stamp
        marker.ns = "rover"
        marker.id = 0
        marker.type = Marker.CUBE       # could also use ARROW
        marker.action = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0

        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        marker.pose.orientation.x = qx
        marker.pose.orientation.y = qy
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw

        # Size of rectangle (in meters)
        marker.scale.x = 1.0   # length
        marker.scale.y = 0.6   # width
        marker.scale.z = 0.2   # height

        # Color (opaque)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.robot_pub.publish(marker)

        # -----------------------
        # Path / trail
        # -----------------------
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = stamp
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        self.path_msg.header.stamp = stamp
        self.path_msg.poses.append(pose)

        # keep path length under control
        if len(self.path_msg.poses) > self.max_points:
            self.path_msg.poses.pop(0)

        self.path_pub.publish(self.path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PathVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()