import numpy as np
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from ublox_ubx_msgs.msg import UBXNavPVT



class Nav(Node):
    def __init__(self):
        # Initialize node
        super().__init__('navigation')

        self.waypoint_sub = self.create_subscription(Float64MultiArray, '/waypoint', self.waypoint_cb, 10)
        self.current_pos_sub = self.create_subscription(UBXNavPVT, '/rover1/ubx_nav_pvt', self.current_pos_cb, 10)
        self.heading_sub = self.create_subscription(Float32, '/heading', self.heading_cb, 10)
        
        # Swerve publisher
        self.drive_pub = self.create_publisher(Float32MultiArray, "/swerve", 10)

        self.target_lat = None
        self.target_lon = None
        self.current_lat = None
        self.current_lon = None
        self.heading = None

        self.MAX_FORWARD_SPEED = 1.0
        self.KP_TURN = 0.02

        # SLEW control scheme
        self.current_linear_x = 0.0
        self.current_rot_right = -1.0
        self.current_rot_left = -1.0

        self.last_gps_time = self.get_clock().now()
        self.last_heading_time = self.get_clock().now()
        
        self.TELEMETRY_TIMEOUT = 5.0 # Seconds

        # controls output at 10 hz
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("GPS Waypoint Navigator initialized...")
        
    
    def waypoint_cb(self, msg):
        self.target_lat, self.target_lon = msg.data[0], msg.data[1] # lat, lon

    def current_pos_cb(self, msg):
        self.last_gps_time = self.get_clock().now()
        current_lat = msg.lat * 1e-7
        current_lon = msg.lon * 1e-7
        
        if self.current_lat is None:
            # First fix
            self.current_lat = current_lat
            self.current_lon = current_lon
        else:
            alpha = 0.8
            self.current_lat = (alpha * current_lat) + ((1.0 - alpha) * self.current_lat)
            self.current_lon = (alpha * current_lon) + ((1.0 - alpha) * self.current_lon)


    def heading_cb(self, msg):
        self.last_heading_time = self.get_clock().now()
        self.heading = msg.data
     
    def haversine_distance(self, lat1, lon1, lat2, lon2):
        """Calculate distance in meters between two GPS coordinates."""
        R = 6378137.0 # Earth radius (meters)
        lat1_r, lon1_r = math.radians(lat1), math.radians(lon1)
        lat2_r, lon2_r = math.radians(lat2), math.radians(lon2)
        
        dlat = lat2_r - lat1_r
        dlon = lon2_r - lon1_r
        
        a = math.sin(dlat/2)**2 + math.cos(lat1_r) * math.cos(lat2_r) * math.sin(dlon/2)**2
        c = 2 * math.asin(math.sqrt(a))
        return R * c

    def calculate_bearing(self, lat1, lon1, lat2, lon2):
        """Compute the compass bearing to get from pos1 to pos2."""
        lat1_r = math.radians(lat1)
        lat2_r = math.radians(lat2)
        dlon = math.radians(lon2 - lon1)
        
        y = math.sin(dlon) * math.cos(lat2_r)
        x = math.cos(lat1_r) * math.sin(lat2_r) - math.sin(lat1_r) * math.cos(lat2_r) * math.cos(dlon)
        
        bearing = math.atan2(y, x)
        return (math.degrees(bearing) + 360) % 360
    
    def check_telemetry(self):
        if self.current_lat is None or self.target_lat is None or self.heading is None:
            return False
        
        now = self.get_clock().now()
        gps_age = (now - self.last_gps_time).nanoseconds / 1e9
        head_age = (now - self.last_heading_time).nanoseconds / 1e9

        if gps_age > self.TELEMETRY_TIMEOUT or head_age > self.TELEMETRY_TIMEOUT:
            self.get_logger().warn(f"Telemetry lost. Last known GPS: {gps_age:.2f} seconds ago, Last known Heading age: {head_age:.2f} seconds ago", throttle_duration_sec=2.0)
            return False
        
        return True
    
    def control_loop(self):
        target_linear_x = 0.0
        target_rot_left = -1.0
        target_rot_right = -1.0
        if not self.check_telemetry():
            # If we lose GPS, then our target will be 0, let SLEW smoothly decelerate
            pass
        else:
            # We have normal GPS data, execute funcs

            dist_to_target = self.haversine_distance(
                self.current_lat, self.current_lon, 
                self.target_lat, self.target_lon
            )
            
            target_bearing = self.calculate_bearing(
                self.current_lat, self.current_lon,
                self.target_lat, self.target_lon
            )

            # Calculate heading error [-180, 180] degrees
            error = (target_bearing - self.heading + 180) % 360 - 180
            
            target_rot_left = -1.0
            target_rot_right = -1.0

            rotation_effort = min(abs(error) / 45.0, 1.0)

            if error > 4.0:
                target_rot_right = -1.0 + (rotation_effort * 2.0)
            elif error < -4.0:
                target_rot_left = -1.0 + (rotation_effort * 2.0) # 2.0 for scaling - eg 90 degree turn, our rotation will be 1.0, and our rot effort will be -1.0, 1.0 - but this should keep going down
            else:
                pass # 4 degree deadband for over/under corrections, should be eh eh

            DECEL_DIST_M = 3.0
            base_target_speed = min(dist_to_target / DECEL_DIST_M, self.MAX_FORWARD_SPEED)

            alignment_factor = 1.0 - rotation_effort # If max rotation (i.e 1.0), then forward speed is exactly 0, if rotation is 0.0, forward speed is exactly 1.0
            target_linear_x = base_target_speed * alignment_factor
        
        # SLEW RATE LIMITER - smooth swerve acceleration and deceleration

        dt = 0.1 # Loop runs at 10 Hz so we have 0.1 seconds per tick
        LIN_ACCEL_RATE = 0.5 # Max units per second to speed up
        LIN_DECEL_RATE = 1.0 # Max units per second to slowdown - currently dominates over accel

        # Rotational acceleration
        # range of -1.0 to 1.0, so change is of 2 units.
        # Having an acceleration rate of 2.0 units/second would mean it would take 1 second to go from neutral to full turn
        ROT_ACCEL_RATE = 0.8 # Maybe slow it down so it takes 4 seconds to go to full turn?

        slew = lambda curr, target, step_acc, step_dec: min(curr + step_acc, target) if target > curr else max(curr - step_dec, target)
        self.current_linear_x = slew(self.current_linear_x, target_linear_x, LIN_ACCEL_RATE * dt, LIN_DECEL_RATE * dt)
        self.current_rot_left = slew(self.current_rot_left, target_rot_left, ROT_ACCEL_RATE * dt, ROT_ACCEL_RATE * dt)
        self.current_rot_right = slew(self.current_rot_right, target_rot_right, ROT_ACCEL_RATE * dt, ROT_ACCEL_RATE * dt)
        
        cmd = [float(self.current_linear_x), 0.0, float(self.current_rot_left), float(self.current_rot_right)]
        msg = Float32MultiArray()
        msg.data = cmd
        self.drive_pub.publish(msg)


    def stop_rover(self):
        msg = Float32MultiArray()
        msg.data = [0.0, 0.0, -1.0, -1.0]
        self.drive_pub.publish(msg)
                
            
def main(args=None):
    rclpy.init(args=args)
    node = Nav()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_rover()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
