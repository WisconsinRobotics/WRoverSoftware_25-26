import math
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Float64
from std_msgs.msg import Float32
from ublox_ubx_msgs.msg import UBXNavPVT
import message_filters

class HeadingCalculator(Node):
    def __init__(self):
        super().__init__('heading_calculator')
        
        self.cb_group = ReentrantCallbackGroup()

        self.sub_rover1 = message_filters.Subscriber(self, UBXNavPVT, '/rover1/ubx_nav_pvt')
        self.sub_rover2 = message_filters.Subscriber(self, UBXNavPVT, '/rover2/ubx_nav_pvt')
        
        self.sub_imu = self.create_subscription(
            Float64, 'heading',
            self.imu_callback, 10,
            callback_group=self.cb_group
        )
        
        self.publisher_ = self.create_publisher(Float32, '/heading', 10)

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_rover1, self.sub_rover2], queue_size=10, slop=0.1
        )
        self.ts.registerCallback(self.synced_gps_callback)

        self.latest_raw_imu = 0.0
        self.imu_offset     = 0.0
        self.has_offset     = False

    def synced_gps_callback(self, msg1, msg2):
        rover1_isRTK = (msg1.carr_soln.status == 2)
        rover2_isRTK = (msg2.carr_soln.status == 2)

        if not (rover1_isRTK and rover2_isRTK):
            return

        lat1 = math.radians(msg1.lat / 1e7)
        lon1 = math.radians(msg1.lon / 1e7)
        lat2 = math.radians(msg2.lat / 1e7)
        lon2 = math.radians(msg2.lon / 1e7)

        heading_rad = math.atan2((lat2 - lat1), (lon2 - lon1) * math.cos(lat1))
        # If rover1 is LEFT antenna, rover2 is RIGHT antenna:
        # vector points RIGHT - forward is 90° counter-clockwise
        MOUNTING_OFFSET_DEG = -90.0

        # If rover1 is RIGHT antenna, rover2 is LEFT antenna:
        # vector points LEFT - forward is 90° clockwise  
        MOUNTING_OFFSET_DEG = +90.0

        # Apply it here:
        gps_heading = (450.0 - math.degrees(heading_rad) + MOUNTING_OFFSET_DEG) % 360.0

        error = (gps_heading - self.latest_raw_imu + 180.0) % 360.0 - 180.0
        self.imu_offset = error
        self.has_offset = True

    def imu_callback(self, msg):
        self.latest_raw_imu = float(msg.data)

        if not self.has_offset:
            # Publish raw IMU until we get RTK lock rather than publishing nothing
            out = Float32()
            out.data = self.latest_raw_imu
            self.publisher_.publish(out)
            return

        corrected = (self.latest_raw_imu + self.imu_offset) % 360.0
        out = Float32()
        out.data = corrected
        self.publisher_.publish(out)


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