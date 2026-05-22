#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray

import serial


class LEDSerialNode(Node):

    def __init__(self):
        super().__init__('led_serial_node')

        # Change this to your Arduino serial port
        self.port = '/dev/ttyACM0'

        # Open serial connection
        self.ser = serial.Serial(self.port, 9600, timeout=1)

        self.get_logger().info(f'Connected to {self.port}')

        # ROS2 subscriber
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'led_color',
            self.color_callback,
            10
        )

    def color_callback(self, msg):
        if len(msg.data) < 3:
            self.get_logger().warn('Need 3 values: R G B')
            return

        # Convert and clamp values
        r = int(max(0, min(255, msg.data[0])))
        g = int(max(0, min(255, msg.data[1])))
        b = int(max(0, min(255, msg.data[2])))

        # XOR checksum
        crc = r ^ g ^ b

        # Send bytes
        packet = bytes([r, g, b, crc])
        self.ser.write(packet)

        self.get_logger().info(f'Sent RGB: {r}, {g}, {b}')

    def destroy_node(self):
        self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = LEDSerialNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
