#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

def normalize_angle(angle):
    """Wrap angle to [-pi, pi]."""
    a = math.fmod(angle + math.pi, 2.0 * math.pi)
    if a < 0.0:
        a += 2.0 * math.pi
    return a - math.pi

class SimRover(Node):
    def __init__(self):
        super().__init__('sim_rover')

        # Publishers: "fake" localization & IMU
        self.loc_pub = self.create_publisher(Float32MultiArray, 'localization_info', 10)
        self.imu_pub = self.create_publisher(Float32MultiArray, 'imu_info', 10)

        # Subscriber: your drive commands
        self.cmd_sub = self.create_subscription(
            Float32MultiArray,
            '/swerve',
            self.cmd_callback,
            10
        )

        # --- initial state (choose what makes sense for your mission) ---
        # For example: start at (20, 0) facing +X
        self.x = 10.0
        self.y = 0.0
        self.yaw = math.pi / 2   # radians, facing +y

        # current commanded body-frame velocities
        # vx: forward (m/s), vy: left/right (m/s), omega: yaw rate (rad/s)
        self.vx_cmd = 0.0
        self.vy_cmd = 0.0
        self.omega_cmd = 0.0

        # integration timestep
        self.dt = 0.02  # 50 Hz

        self.timer = self.create_timer(self.dt, self.update)

    # ---------------------------
    #  Callback for /swerve
    # ---------------------------
    def cmd_callback(self, msg: Float32MultiArray):
        data = msg.data

        # !!! VERY IMPORTANT !!!
        # Adjust this mapping to match what your drive_logic actually sends.
        #
        # Example assumption:
        #  data[0] = vx_forward (m/s)
        #  data[1] = vy_left    (m/s)  # if you don't use lateral motion, set this to 0
        #  data[2] = omega      (rad/s)
        #
        # If you only send [linear_speed, angular_speed], then set vy_cmd = 0
        try:
            self.vy_cmd = float(data[0])
            self.vx_cmd = float(data[1])
            self.omega_cmd = float(data[2]) if float(data[3]) == 0 else -1 * float(data[3])
        except (IndexError, ValueError):
            self.get_logger().warn(f"Received malformed /swerve command: {float(data[0])}, {float(data[1])}, {float(data[2])}, {float(data[3])}")

    # ---------------------------
    #  Timer: integrate motion
    # ---------------------------
    def update(self):
        # integrate kinematics with current command
        # commands are assumed in BODY frame
        vx = self.vx_cmd
        vy = self.vy_cmd
        omega = self.omega_cmd

        # Transform body-frame velocity to world-frame
        cos_y = math.cos(self.yaw)
        sin_y = math.sin(self.yaw)

        x_dot = vx * cos_y - vy * sin_y
        y_dot = vx * sin_y + vy * cos_y
        yaw_dot = omega

        self.x += x_dot * self.dt
        self.y += y_dot * self.dt
        self.yaw = normalize_angle(self.yaw + yaw_dot * self.dt)
        angle = normalize_angle(math.atan2(self.y, self.x))

        # -----------------------
        # publish Localization_info
        # -----------------------
        loc_msg = Float32MultiArray()
        # Adjust to whatever layout your drive_logic expects.
        # Example: [x, y, yaw]
        loc_msg.data = [self.x, self.y]
        self.loc_pub.publish(loc_msg)

        # -----------------------
        # publish imu_info
        # -----------------------
        imu_msg = Float32MultiArray()
        # Example layout: [vx_body, vy_body, omega, yaw]
        imu_msg.data = [vx, vy, angle, self.yaw]
        self.imu_pub.publish(imu_msg)
        self.get_logger().info(f"coordinates: ({self.x}, {self.y})\nangle: {angle}, orientation: {self.yaw}")

def main(args=None):
    rclpy.init(args=args)
    node = SimRover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()