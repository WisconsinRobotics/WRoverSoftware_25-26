import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry

class FusionNode(Node):
	def __init__(self):
        	super().__init__('fusion_node')

		self.imu_state = {
			'quaternion' : np.zeros(4),
			'angular_velocity' : np.zeros(3),
			'linear_acceleration' : np.zeros(3)
		}

		self.gnss_ref = None # will store initial lat/lon for later conversions
		self.gnss_state = np.zeros(2)

		self.state_vector = np.zeros(10)
		self.P = np.eye(10) #covariance
		self.Q = np.eye(10) * 0.01 #process noise
		self.R = np.eye(3) * 5.0 # GNSS measurement noise
		self.last_time = None

        	self.imu_sub = self.create_subscription(Imu, '/imu_quat_data', self.imu_callback, 10)
        	self.gnss_sub = self.create_subscription(NavSatFix, 'fix', self.gnss_callback, 10)
        	self.pub = self.create_publisher(Odometry, '/fused_data', 10)


	def imu_callback(self, msg: Imu):
		self.imu_state['quaternion'] = np.array([
			msg.orientation.x,
			msg.orientation.y,
			msg.orientation.z,
			msg.orientation.w
		])

		self.imu_state['angular_velocity'] = np.array([
		        msg.angular_velocity.x,
			msg.angular_velocity.y,
        		msg.angular_velocity.z
    		])

		self.imu_state['linear_acceleration'] = np.array([
        		msg.linear_acceleration.x,
        		msg.linear_acceleration.y,
        		msg.linear_acceleration.z
    		])

		now = self.get_clock().now().nanoseconds * 1e-9
		dt = now - self.last_time if self.last_time is not None
		self.last_time = now

		self.fusion(dt)


	def gnss_callback(self, msg: NavSatFix):
		if self.gnss_ref is None:
			self.gnss_ref = np.array([msg.latitude, msg.longitude])
		self.gnss_state = np.array([msg.latitude, msg.longitude])

	def fusion(self, dt):
		self.update_position_state(dt)
		self.update_covariance(dt)
		K = self.compute_kalman_gain(H, R)
		#correct with gnss measurements here: 

	def update_position_state(self, dt):
		pass

	def update_covariance(self, dt):
		pass

	def compute_kalman_gain(self, H, R):
		pass

	def to_cartesian(self):
	#converts gnss to local cartesian frame
		pass

	def quarternion_to_rotation(self):
		pass

	def publish_fused_state(self):
		pass

def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    rclpy.spin(node)
    rclpy.shutdown()
