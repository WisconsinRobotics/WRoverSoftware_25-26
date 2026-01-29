import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
# Helper for quaternion conversion
from tf_transformations import euler_from_quaternion

class OdomDegreePrinter(Node):
    def __init__(self):
        super().__init__('odom_degree_printer')
        self.subscription = self.create_subscription(Odometry, '/fused_data', self.callback, 1)

    def callback(self, msg):
        # 1. Convert Orientation (Quaternion -> Euler Radians -> Degrees)
        q = msg.pose.pose.orientation
        quat_list = [q.x, q.y, q.z, q.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat_list)
        
        r_deg = math.degrees(roll)
        p_deg = math.degrees(pitch)
        y_deg = math.degrees(yaw)

        # 2. Convert Angular Velocity (Radians/s -> Degrees/s)
        ang_vel = msg.twist.twist.angular
        av_x = math.degrees(ang_vel.x)
        av_y = math.degrees(ang_vel.y)
        av_z = math.degrees(ang_vel.z)

        self.get_logger().info(
            f"\n--- ODOMETRY (DEGREES) ---\n"
            f"Orientation (RPY): [{r_deg:.2f}°, {p_deg:.2f}°, {y_deg:.2f}°]\n"
            f"Ang. Velocity:    [{av_x:.2f}°/s, {av_y:.2f}°/s, {av_z:.2f}°/s]\n"
            f"Position (XYZ):   [{msg.pose.pose.position.x:.2f}m, {msg.pose.pose.position.y:.2f}m, {msg.pose.pose.position.z:.2f}m]"
        )

def main():
    rclpy.init()
    node = OdomDegreePrinter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()