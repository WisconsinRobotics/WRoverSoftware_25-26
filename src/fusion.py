import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry

class FusionNode(Node):
	def __init__(self):
        	super().__init__('fusion_node')

		self.imu_state = {
			'quaternion' : np.array([0.0, 0.0, 0.0, 1.0])
			'angular_velocity' : np.zeros(3),
			'linear_acceleration' : np.zeros(3)
		}

		self.gnss_ref = None # will store initial lat/lon for later conversions
		self.gnss_state = np.zeros(2)

		self.state_vector = np.zeros(10)
		self.H = np.zeros((3,10)) #Observation
                H[0,0] = 1
                H[1,1] = 1
                H[2,2] = 1
		self.P = np.eye(10) * 0.1  #covariance
		self.Q = np.eye(10) * 0.01 #process noise, need to measure from the robot
		self.R = np.eye(3) * 5.0 # GNSS measurement noise, 5.0 is placeholder for how many meters^2 accurate the gnss is, need to measure
		self.last_time = None
		self.initialized = False

        	self.imu_sub = self.create_subscription(Imu, '/imu_quat_data', self.imu_callback, 10)
        	self.gnss_sub = self.create_subscription(NavSatFix, 'fix', self.gnss_callback, 10)
        	self.pub = self.create_publisher(Odometry, '/fused_data', 10)

		self.timer = self.create_timer(0.02, self.fusion_timer_callback)

	def fusion_timer_callback(self):
                now = self.get_clock().now().nanoseconds() * 1e-9
                dt = 0 if self.last_time is not None else now - self.last_time
                self.last_time = now

                self.fusion(dt)

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

	def gnss_callback(self, msg: NavSatFix):
		if self.gnss_ref is None:
			self.gnss_ref = np.array([msg.latitude, msg.longitude])
		self.gnss_state = np.array([msg.latitude, msg.longitude])

	def initialize_state(self):
		if self.initialized:
			return

		if np.linalg.norm(self.imu_state['quaternion']) == 0:
			return

		if self.gnss_ref is None and np.linalg.norm(self.gnss_state) != 0:
			self.gnss_ref = self.gnss_state.copy()
			return

		if self.gnss_ref is not None:
			position = self.to_cartesion(self.gnss_state)
		else:
			position = np.zeros(3)

		self.state_vector[0:3] = position
		self.state_vector[3:6] = np.zeros(3)
		self.state_vector[6:10] = self.imu_state['quaternion']

		self.initialized = True
		self.last_time = now


	def fusion(self, dt):
		self.initialize_state()
		if np.linalg.norm(self.state_vector) == 0:
			return

		self.update_position_state(dt)
		self.update_covariance(dt)

		K = self.compute_kalman_gain()
		#correct with gnss measurements here:

	def update_position_state(self, dt):
		px, py, pz = self.state_vector[0:3]
		vx, vy, vz = self.state_vector[3:6]
		q = self.state_vector[6:10]

		acceleration_body = self.imu_state['linear_acceleration']

		rotation = self.quaternion_to_rotation(q)
		acceleration_world = rotation @ acceleration_body

		gravity = np.array([0, 0, 9.8])
		acceleration_world -= gravity

		vx_updated = vx + acceleration_world[0] * dt
		vy_updated = vy + acceleration_world[1] * dt
		vz_updated = vz + acceleration_world[2] * dt

		px_updated = px + vx * dt + 0.5 * acceleration_world[0] * dt * dt
		py_updated = py + vy * dt + 0.5 * acceleration_world[1] * dt * dt
		pz_updated = pz + vz * dt + 0.5 * acceleration_world[2] * dt * dt

		self.state_vector[0:3] = [px_updated, py_updated, pz_updated]
		self.state_vector[3:6] = [vx_updated, vy_updated, vz_updated]

	def update_covariance(self, dt):
		F = np.eye(10) #Jacobian of IMU state
		F[0, 3] = dt
		F[1, 4] = dt
		F[2, 5] = dt

		self.P  = F @ self.P  @ F.transpose() + Q

	def compute_kalman_gain(self):
		S = self.H @ self.P @ self.H.transpose() + self.R
		return self.P @ self.H.transpose() @ np.linalg.inv(S)

	def to_cartesian(self):
	#converts gnss to local cartesian frame
		pass

	def quarternion_to_rotation(self, q):
		qx, qy, qx, qw = q

		xx = qx * qx
		yy = qy * qy
		zz = qz * qz
		xy = qx * qy
		xz = qx * qz
		yz = qy * qz
		wx = qw * qx
		wy = qw * qy
		wz = qw * qz

		rotation = np.array([
			[1 - 2 * (yy + zz),	2 * (xy - wz),	2 * (xz + wy)],
			[2 * (xy + wz),		1 - 2 * (xx + zz), 	2 * (yz - wx)],
			[2 * (xz - wy),		2 * (yz + wx), 	1 - 2 * (xx + yy)]
		])

		return rotation

	def publish_fused_state(self):
		pass

def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    rclpy.spin(node)
    rclpy.shutdown()
