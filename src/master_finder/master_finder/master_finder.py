# Core logic stuff
import math
from geographiclib.geodesic import Geodesic

# ROS stuff
import rclpy
from rclpy.node import Node
from ublox_ubx_msgs.msg import UBXNavPVT
from geographic_msgs.msg import GeoPoint, GeoPath, GeoPoseStamped
from std_msgs.msg import Float64MultiArray, Header

"""
To solve the blindspot issue have one camera following the path, and another camera looking inwards.
"""


# =====================================================
# PATH GENERATION CLASS (For a particular target point)
# =====================================================
class PathGeneration:
    """
        Uses WGS84 geodesic search patterns to generate highly accurate archimedean spiral search patterns for a path
        Takes as parameters
    """
    def __init__(self, target_lat, target_lon, prev_lat, prev_lon, r_start : float = 0.0, r_stop : float = 20.0):
        self.target_lat = target_lat
        self.target_lon = target_lon
        self.current_lat = prev_lat
        self.current_lon = prev_lon

        self.r_start = r_start # start spiral at center will change dependent on mission
        self.r_max = r_stop # max radius in meters
        self.loop_spacing = 4.0 # distance between the spiral arms

        self.resolution_meters = 0.5 # Distance between GPS waypoints is now half a meter

        self.geodesic = Geodesic.WGS84

    def generate_search_pattern(self):
        """
            Builds an inside to out archimedean spiral
        """
        path_msg = GeoPath()

        # Find initial bearing from target to current rover position. Spiral begins outward from angle rover approaches from.
        if self.target_lat == self.current_lat and self.target_lon == self.current_lon:
            base_azimuth = 0.0
        else:
            inv_result = self.geodesic.Inverse(self.target_lat, self.target_lon, self.current_lat, self.current_lon) # Solve the inverse geodesic problem
            base_azimuth = inv_result['azi1'] # The bearing from target to current rover pos

        # Archimedean spiral with a 4m distance between the arms
        a = self.r_start
        b = self.loop_spacing / (2 * math.pi)

        # Total angle to reach r_max (upper bound)
        theta_max = ((self.r_max - a) / b) + 1
        theta_min = 0.0
        pose = self._create_geo_pose(self.target_lat, self.target_lon)
        path_msg.poses.append(pose)
        
        if self.r_start != 0.0:
            theta_min = -2
            
        while theta_min <= theta_max:
            r = a + b * theta_min # radius of rotation
            current_az = base_azimuth + math.degrees(theta_min) # bearing corrected for elliptoids
            direct_res = self.geodesic.Direct(self.target_lat, self.target_lon, current_az, r)
            pose = self._create_geo_pose(direct_res['lat2'], direct_res['lon2'])
            path_msg.poses.append(pose)

            r_safe = max(r, 0.1)
            d_theta = self.resolution_meters / r_safe
            theta_min += d_theta
            
        return path_msg
    
    def _create_geo_pose(self, lat, lon):
        pose = GeoPoseStamped()
        pose.pose.position.latitude = lat
        pose.pose.position.longitude = lon
        pose.pose.position.altitude = 0.0

        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0 
        return pose


class MasterFinder(Node):
    """
        ROS 2 Node that orchestrates the search pattern algorithms
    """

    def __init__(self):
        super().__init__('master_finder')

        self.current_lat = None
        self.current_lon = None

        self.target_sub = self.create_subscription(
            Float64MultiArray, 
            'spiral_request', 
            self.target_request_callback, 
            10
        )

        self.path_pub = self.create_publisher(GeoPath, 'spiral_path', 10)
        self.get_logger().info("MasterFinder initialized. Waiting for target request...")

    def target_request_callback(self, msg : Float64MultiArray):
        "Triggered when the state machine will call this to generate paths"
        target_lat = msg.data[0]
        target_lon = msg.data[1]
        prev_lat = msg.data[2]
        prev_lon = msg.data[3]
        r_start = msg.data[4]
        r_stop = msg.data[5]

        
        self.get_logger().info(f"Generating archimedean spiral path for target [{target_lat}, {target_lon}]")
        
        generator = PathGeneration(
            target_lat=target_lat,
            target_lon=target_lon,
            prev_lat = prev_lat,
            prev_lon=prev_lon,
            r_start=r_start,
            r_stop=r_stop
        )

        # Get GeoPath message of GeoPoseStamped waypoints
        search_path_msg = generator.generate_search_pattern()
        search_path_msg.header = Header()
        search_path_msg.header.stamp = self.get_clock().now().to_msg()
        search_path_msg.header.frame_id = 'map'
        
        self.path_pub.publish(search_path_msg)
        self.get_logger().info(f"Published search spiral with {len(search_path_msg.poses)} active GeoPoseStamped waypoints.")


def main(args=None):
    rclpy.init(args=args)
    node = MasterFinder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()