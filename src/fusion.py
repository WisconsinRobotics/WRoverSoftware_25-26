import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry
from pyproj import CRS, Transformer

class FusionNode(Node):
	def __init__(self):
		super().__init__('fusion_node')

		self.imu_state = {
			'quaternion' : np.array([0.0, 0.0, 0.0, 1.0]),
			'angular_velocity' : np.zeros(3),
			'linear_acceleration' : np.zeros(3)
		}

		self.gnss_ref = None # will store initial lat/lon for later conversions
		self.gnss_state = np.zeros(2)

		self.state_vector = np.zeros(10)
		self.H = np.zeros((3,10)) #Observation
		self.H[0,0] = 1
		self.H[1,1] = 1
		self.H[2,2] = 1
		self.P = np.eye(10) * 0.1  #covariance
		self.Q = np.eye(10) * 0.01 #process noise, need to measure from the robot
		self.R = np.eye(3) * 5.0 # GNSS measurement noise, 5.0 is placeholder for how many meters^2 accurate the gnss is, need to measure
		self.R[2,2] = 1e6 #makes it so z is largely ignored as we arent measuring it too closely for now
		self.last_time = None
		self.initialized = False

		self.imu_sub = self.create_subscription(Imu, '/imu_quat_data', self.imu_callback, 10)
		self.gnss_sub = self.create_subscription(NavSatFix, 'fix', self.gnss_callback, 10)
		self.pub = self.create_publisher(Odometry, '/fused_data', 10)

		self.timer = self.create_timer(0.02, self.fusion_timer_callback)

		self.utm_transformer = None
		self.utm_crs = None
		self.utm_zone = None
		self.initial_yaw = None


	def fusion_timer_callback(self):
		now = self.get_clock().now().nanoseconds() * 1e-9
		dt = 0 if self.last_time is None else now - self.last_time
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

		if self.gnss_ref is not None:
			position = self.to_cartesian(self.gnss_state)
		else:
			position = np.zeros(3)

		self.state_vector[0:3] = position
		self.state_vector[3:6] = np.zeros(3)
		self.state_vector[6:10] = self.imu_state['quaternion']

		self.initialized = True
		self.last_time = 0


	def fusion(self, dt):
		self.initialize_state()
		if not self.initialized:
			return

		self.state_vector[6:10] = self.imu_state['quaternion']
		self.update_position_state(dt)
		self.update_covariance(dt)

		self.gnss_correction()

		self.publish_fused_state()

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

		self.P  = F @ self.P  @ F.transpose() + self.Q

	def compute_kalman_gain(self):
		S = self.H @ self.P @ self.H.transpose() + self.R
		return self.P @ self.H.transpose() @ np.linalg.inv(S)

	def gnss_correction(self):
		if self.gnss_ref is None or np.linalg.norm(self.gnss_state) == 0:
			return

		current = self.to_cartesian(self.gnss_state)
		expected_current  = self.H @ self.state_vector
		error = current - expected_current

		K = self.compute_kalman_gain()

		self.state_vector = self.state_vector + (K @ error)

		I = np.eye(self.P.shape[0])
		self.P = (I - K @ self.H) @ self.P

		#renormalize quartinion
		q = self.state_vector[6:10]
		qn = np.linalg.norm(q)
		if qn > 1e-12:
			self.state_vector[6:10] = q / qn

	def to_cartesian(self, latlon):
	#converts gnss to local cartesian frame
	#x-> East
	#y-> North
		lat, lon = latlon
		lat0, lon0 = self.gnss_ref

		R = 6378137.0 #radius of earth in meters

		lat_rad = np.deg2rad(lat)
		lon_rad = np.deg2rad(lon)
		lat0_rad = np.deg2rad(lat0)
		lon0_rad = np.deg2rad(lon0)

		dlat = lat_rad - lat0_rad
		dlon = lon_rad - lon0_rad

		x = dlon * np.cos(lat0_rad) * R
		y = dlat * R
		z = 0

		return np.array([x, y, z])

	def quaternion_to_rotation(self, q):
		qx, qy, qz, qw = q

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
		if not self.initialized or self.gnss_ref is None:
			return

		R = 6378137.0

		x_local, y_local, _ = self.state_vector[0:3]

		lat0, lon0 = self.gnss_ref
		lat0_rad = np.deg2rad(lat0)

		lat = lat0 + (y_local / R) * (180.0 / np.pi)
		lon = lon0 + (x_local / (R * np.cos(lat0_rad))) * (180.0 / np.pi)

		# Ensure UTM transformer exists (based on start zone)
		self.ensure_utm(lat0, lon0)

		# Convert to UTM (meters)
		easting, northing = self.utm_transformer.transform(lon, lat)

		# Yaw from quaternion
		qx, qy, qz, qw = self.state_vector[6:10]
		yaw = np.arctan2(
			2.0 * (qw * qz + qx * qy),
			1.0 - 2.0 * (qy * qy + qz * qz)
		)

		if self.initial_yaw is None:
			self.initial_yaw = yaw
		yaw_rel = yaw - self.initial_yaw
		yaw_rel = (yaw_rel + np.pi) % (2.0 * np.pi) - np.pi  # wrap [-pi, pi]


		msg = Odometry()
		msg.header.stamp = self.get_clock().now().to_msg()
		msg.header.frame_id = f"utm_zone_{self.utm_zone}"
		msg.child_frame_id = "base_link"

		msg.pose.pose.position.x = float(easting)
		msg.pose.pose.position.y = float(northing)
		msg.pose.pose.position.z = 0.0

		# Quaternion for yaw_rel (roll=pitch=0)
		msg.pose.pose.orientation.z = float(np.sin(yaw_rel / 2.0))
		msg.pose.pose.orientation.w = float(np.cos(yaw_rel / 2.0))

		self.pub.publish(msg)


	def ensure_utm(self, lat, lon):
		# Build transformer once (based on start position)
		if self.utm_transformer is not None:
			return

		zone = int(np.floor((lon + 180.0) / 6.0) + 1)
		north = lat >= 0.0
		epsg = 32600 + zone if north else 32700 + zone  # WGS84 UTM

		self.utm_zone = zone
		self.utm_crs = CRS.from_epsg(epsg)
		self.utm_transformer = Transformer.from_crs(
		CRS.from_epsg(4326),  # WGS84 lat/lon
		self.utm_crs,
		always_xy=True        # expects lon, lat
		)


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    rclpy.spin(node)
    print(self.state_vector[0:10])
    rclpy.shutdown()
