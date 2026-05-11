import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from ublox_ubx_msgs.msg import UBXNavPVT

class HeadingCalculator(Node):
    def __init__(self):
        super().__init__('heading_calculator')
        
        self.sub_rover1 = self.create_subscription(
            UBXNavPVT,
            '/rover1/ubx_nav_pvt',   
            self.rover1_callback,
            10)
        
        self.sub_rover2 = self.create_subscription(
            UBXNavPVT,
            '/rover2/ubx_nav_pvt',   
            self.rover2_callback,
            10)
            
        self.publisher_ = self.create_publisher(Float32, '/heading', 10)
        
        # Variables to store the latest coordinates
        self.rover1_lat = None
        self.rover1_lon = None
        self.rover2_lat = None
        self.rover2_lon = None

    def rover1_callback(self, msg):
        self.rover1_lat = msg.lat / 1e7
        self.rover1_lon = msg.lon / 1e7
        self.calculate_heading()

    def rover2_callback(self, msg):
        self.rover2_lat = msg.lat / 1e7
        self.rover2_lon = msg.lon / 1e7
        self.calculate_heading()

    def calculate_heading(self):
        # Don't do math until we have received at least one message from BOTH antennas
        if None in[self.rover1_lat, self.rover1_lon, self.rover2_lat, self.rover2_lon]:
            return

        # Convert everything to radians for Python's math library
        lat1 = math.radians(self.rover1_lat)
        lon1 = math.radians(self.rover1_lon)
        lat2 = math.radians(self.rover2_lat)
        lon2 = math.radians(self.rover2_lon)

        # This calculates the angle of the vector pointing from Rover 1 TO Rover 2
        heading_rad = math.atan2((lat2 - lat1), ((lon2 - lon1) * math.cos(lat1)))
        
        # Math angle: 0=East, 90=North. Compass angle: 0=North, 90=East.
        math_degrees = math.degrees(heading_rad)
        compass_heading = (450.0 - math_degrees) % 360.0

        msg = Float32()
        msg.data = compass_heading
        self.publisher_.publish(msg)
        

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