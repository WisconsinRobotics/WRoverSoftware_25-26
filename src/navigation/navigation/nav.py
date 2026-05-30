import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float32
from ublox_ubx_msgs.msg import UBXNavPVT

class Nav(Node):
    def __init__(self, target_lat, target_lon):
        # Initialize node
        super().__init__('navigation')

        self.current_pos_sub = self.create_subscription(UBXNavPVT, '/rover1/ubx_nav_pvt', self.current_pos_cb, 10)
        self.heading_sub = self.create_subscription(Float32, '/heading', self.heading_cb, 10)
        
        # Swerve publisher
        self.drive_pub = self.create_publisher(Float32MultiArray, "/swerve", 10)

        # Create led publisher
        self.led_publisher = self.create_publisher(Float32MultiArray, 'led', 1)
        self.led_msg = Float32MultiArray()
        
        # Set led to red at startup
        self.led_msg.data = [255.0, 0.0, 0.0]
        self.led_publisher.publish(self.led_msg)

        # Set user inputted coordinates
        self.target_lat = target_lat
        self.target_lon = target_lon
        
        self.current_lat = None
        self.current_lon = None
        self.heading = None

        # SLOW and steady base speeds
        self.MAX_FORWARD_SPEED = 0.8 
        self.KP_TURN = 0.015

        self.last_gps_time = self.get_clock().now()
        self.last_heading_time = self.get_clock().now()
        
        self.TELEMETRY_TIMEOUT = 5.0 # Seconds

        # LED Flashing parameters
        self.interval = 0.1
        self.counter = 0

        # controls output at 10 hz
        self.timer = self.create_timer(self.interval, self.control_loop)
        self.get_logger().info(f"GPS Waypoint Navigator initialized! Target: ({self.target_lat}, {self.target_lon})...")

    def current_pos_cb(self, msg):
        self.last_gps_time = self.get_clock().now()
        current_lat = msg.lat * 1e-7
        current_lon = msg.lon * 1e-7
        
        if self.current_lat is None:
            self.current_lat = current_lat
            self.current_lon = current_lon
        else:
            alpha
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
            self.get_logger().warn(f"Telemetry lost. GPS age: {gps_age:.2f}s, Heading age: {head_age:.2f}s", throttle_duration_sec=2.0)
            return False
        
        return True
    
    def control_loop(self):
        if not self.check_telemetry():
            # If we lose GPS, stop immediately
            self.stop_rover()
            return

        dist_to_target = self.haversine_distance(
            self.current_lat, self.current_lon, 
            self.target_lat, self.target_lon
        )
        
        # --- DESTINATION REACHED LOGIC ---
        if dist_to_target < 1.5:
            # Stop moving
            self.stop_rover()
            self.counter += 1
            
            # Flash green at 2 Hz (or requested interval rate)
            if self.counter % int(0.5 / self.interval) == 0:
                # Typecast to list protects from ROS2 Float32MultiArray evaluating as array.array type natively
                if list(self.led_msg.data) != [0.0, 255.0, 0.0]:
                    self.led_msg.data = [0.0, 255.0, 0.0]
                else:
                    self.led_msg.data = [0.0, 0.0, 0.0]
                    
                self.led_publisher.publish(self.led_msg)
            
            self.get_logger().info(f"Target Reached! Distance: {dist_to_target:.2f}m", throttle_duration_sec=1.0)
            return
            

        # --- NORMAL DRIVING LOGIC ---
        target_bearing = self.calculate_bearing(
            self.current_lat, self.current_lon,
            self.target_lat, self.target_lon
        )

        # Calculate heading error [-180, 180] degrees
        error = (target_bearing - self.heading + 180) % 360 - 180
        
        # --- PROPORTIONAL CONTROL (No Smoothing) ---
        
        # 1. Forward Speed Scales inversely with error (if facing away, go slower. Min 0.2 scale)
        speed_factor = max(0.2, 1.0 - (abs(error) / 45.0))
        
        # Also drop speed if we are close to the target distance-wise
        DECEL_DIST_M = 4.0
        dist_factor = min(dist_to_target / DECEL_DIST_M, 1.0)
        
        forward_speed = self.MAX_FORWARD_SPEED * speed_factor * dist_factor

        # 2. Turn Speed Scales proportionally with error
        # Yields a turning effort mapped to your [-1.0, 1.0] expected wheel logic limits
        turn_speed = max(min(abs(error) * self.KP_TURN, 1.3), 0.05) - 1.0
        
        # Base neutrals
        rot_left = -1.0
        rot_right = -1.0

        # Apply deadband of 2 degrees
        if error > 10.0:
            rot_right = turn_speed
        elif error < -10.0:
            rot_left = turn_speed

        cmd = [float(forward_speed), 0.0, float(rot_left), float(rot_right)]
        
        msg = Float32MultiArray()
        msg.data = cmd
        self.drive_pub.publish(msg)

        self.get_logger().info(
            f"Err: {error:.1f}deg | Dist: {dist_to_target:.1f}m | Fwd: {forward_speed:.2f} | Turn: {turn_speed:.2f}"
        )

    def stop_rover(self):
        msg = Float32MultiArray()
        msg.data = [0.0, 0.0, -1.0, -1.0]
        self.drive_pub.publish(msg)
                
            
def main(args=None):
    print("=== GNSS Navigator Initiating ===")
    try:
        t_lat = float(input("Enter target Latitude:  "))
        t_lon = float(input("Enter target Longitude: "))
    except ValueError:
        print("Invalid input. Please enter valid numeric decimal coordinates.")
        return

    rclpy.init(args=args)
    node = Nav(target_lat=t_lat, target_lon=t_lon)

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
