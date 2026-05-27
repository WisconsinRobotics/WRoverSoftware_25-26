import math
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Float64, Float32
from ublox_ubx_msgs.msg import UBXNavPVT


def haversine_distance_m(lat1_rad, lon1_rad, lat2_rad, lon2_rad):
    """Great-circle distance in metres between two points given in radians."""
    R = 6_371_000.0
    dlat = lat2_rad - lat1_rad
    dlon = lon2_rad - lon1_rad
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2) ** 2
    return R * 2.0 * math.asin(math.sqrt(a))


def bearing_deg(lat1_rad, lon1_rad, lat2_rad, lon2_rad):
    """Forward azimuth (0–360°, clockwise from North) between two points given in radians."""
    dlon = lon2_rad - lon1_rad
    x = math.sin(dlon) * math.cos(lat2_rad)
    y = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon)
    return math.degrees(math.atan2(x, y)) % 360.0


class HeadingCalculator(Node):

    # Minimum distance the rover must travel before we trust the GPS heading
    MIN_MOVE_M: float = 1.0

    def __init__(self):
        super().__init__('heading_calculator')
        self.cb_group = ReentrantCallbackGroup()

        # Single RTK receiver
        self.create_subscription(
            UBXNavPVT, '/rover1/ubx_nav_pvt',
            self.gps_callback, 10,
            callback_group=self.cb_group,
        )

        # IMU yaw (degrees, 0–360, CW from North)
        self.create_subscription(
            Float64, 'compass_data_topic',
            self.imu_callback, 10,
            callback_group=self.cb_group,
        )

        self.pub = self.create_publisher(Float32, '/heading', 10)

        # GPS state
        self._prev_lat: float | None = None
        self._prev_lon: float | None = None

        # Calibration state
        self._raw_imu:    float = 0.0
        self._imu_offset: float = 0.0
        self._has_offset: bool  = False

    def gps_callback(self, msg):
        # Require RTK fixed (carr_soln == 2)
        if msg.carr_soln.status != 2:
            return

        lat = math.radians(msg.lat / 1e7)
        lon = math.radians(msg.lon / 1e7)

        # First valid RTK fix – just store it, nothing to compare against yet
        if self._prev_lat is None:
            self._prev_lat, self._prev_lon = lat, lon
            return

        dist = haversine_distance_m(self._prev_lat, self._prev_lon, lat, lon)
        if dist < self.MIN_MOVE_M:
            return  # Haven't moved enough; keep waiting

        gps_heading = bearing_deg(self._prev_lat, self._prev_lon, lat, lon)

        # Signed angular error (–180 … +180)
        error = (gps_heading - self._raw_imu + 180.0) % 360.0 - 180.0
        self._imu_offset = error
        self._has_offset = True

        # Slide the reference forward so the *next* update uses the current pos
        self._prev_lat, self._prev_lon = lat, lon


    def imu_callback(self, msg):
        self._raw_imu = float(msg.data) % 360.0
        print("imu reading = ", self._raw_imu)
        out = Float32()
        if self._has_offset:
            out.data = float((self._raw_imu + self._imu_offset) % 360.0)
        else:
            # No calibration yet – pass raw IMU through so downstream isn't starved
            out.data = self._raw_imu

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = HeadingCalculator()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()