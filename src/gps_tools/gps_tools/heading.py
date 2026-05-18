import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from ublox_ubx_msgs.msg import UBXNavPVT
import message_filters

class HeadingCalculator(Node):
    def __init__(self):
        super().__init__('heading_calculator')
        
        self.sub_rover1 = message_filters.Subscriber(self, UBXNavPVT, '/rover1/ubx_nav_pvt')
        self.sub_rover2 = message_filters.Subscriber(self, UBXNavPVT, '/rover2/ubx_nav_pvt')
        
        self.sub_imu = self.create_subscription(Float64, 'compass_data_topic', self.imu_callback, 10)
            
        self.publisher_ = self.create_publisher(Float32, '/heading', 10)
        
        #Create an ApproximateTimeSynchronizer
        # queue_size = 10
        # slop = 0.1 seconds (maximum time difference between the two messages to consider them a "pair")
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_rover1, self.sub_rover2], queue_size=10, slop=0.1)
        # GPS Variables
        self.rover1_lat = None
        self.rover1_lon = None
        self.rover2_lat = None
        self.rover2_lon = None
        self.rover1_isRTK = False
        self.rover2_isRTK = False
        
        # State Variables
        self.latest_raw_imu = 0.0
        self.imu_offset = 0.0  # The difference between IMU and True North
        self.has_offset = False # Don't publish until we've locked RTK at least once

    def rover1_callback(self, msg):
        self.rover1_lat = msg.lat / 1e7
        self.rover1_lon = msg.lon / 1e7
        self.rover1_isRTK = (msg.carr_soln.status == 2)
        self.update_gps_offset() # Try to update offset when new GPS data arrives

    def rover2_callback(self, msg):
        self.rover2_lat = msg.lat / 1e7
        self.rover2_lon = msg.lon / 1e7
        self.rover2_isRTK = (msg.carr_soln.status == 2)
        self.update_gps_offset() # Try to update offset when new GPS data arrives

    def update_gps_offset(self):
        # ONLY proceed if BOTH antennas have RTK Fixed!
        if not (self.rover1_isRTK and self.rover2_isRTK):
            return

        # Make sure we actually have coordinates
        if None in [self.rover1_lat, self.rover1_lon, self.rover2_lat, self.rover2_lon]:
            return

        # Convert everything to radians
        lat1 = math.radians(self.rover1_lat)
        lon1 = math.radians(self.rover1_lon)
        lat2 = math.radians(self.rover2_lat)
        lon2 = math.radians(self.rover2_lon)

        # Calculate vector from Rover 1 to Rover 2
        heading_rad = math.atan2((lat2 - lat1), ((lon2 - lon1) * math.cos(lat1)))
        
        # Math angle: 0=East, 90=North. Compass angle: 0=North, 90=East.
        math_degrees = math.degrees(heading_rad)
        gps_compass_heading = (450.0 - math_degrees) % 360.0

        # Calculate the shortest angle error between GPS and raw IMU (-180 to 180 degrees)
        # Formula: (Target - Source + 180) % 360 - 180
        error = (gps_compass_heading - self.latest_raw_imu + 180.0) % 360.0 - 180.0
        
        # Update the offset (you could also add a low-pass filter here later if it jitters)
        self.imu_offset = error
        self.has_offset = True

    def imu_callback(self, msg):
        self.latest_raw_imu = msg.data

        # If we have never gotten a good RTK lock, we can't publish a corrected heading yet
        if not self.has_offset:
            return

        # Apply the offset to the fast-arriving IMU data
        corrected_heading = (self.latest_raw_imu + self.imu_offset) % 360.0

        # Publish
        out_msg = Float32()
        out_msg.data = corrected_heading
        self.publisher_.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = HeadingCalculator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()