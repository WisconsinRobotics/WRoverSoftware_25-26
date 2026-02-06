#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class SimTag(Node):
    def __init__(self):
        super().__init__('sim_tag')

        # Publisher: "fake" tag info.
        self.tag_pub = self.create_publisher(Float32MultiArray, 'aruco_results', 10)

        self.timer = self.create_timer(0.02, self.tag_callback)


    def tag_callback(self):
        msg = Float32MultiArray()
        msg.data = [-1.0, 0.0, 0.0]
        self.tag_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimTag()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()