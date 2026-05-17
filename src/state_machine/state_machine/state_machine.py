import os
import rclpy
from rclpy.node import Node
from wr_interfaces.srv import MultiPathPlan
from ublox_ubx_msgs.msg import UBXNavPVT
from geographic_msgs.msg import GeoPoint
from std_msgs.msg import Float32MultiArray
from ament_index_python.packages import get_package_share_directory



class StateMachine(Node):
    def __init__(self):
        # Initialization
        super().__init__('state_machine')
        
        # Create subscribers for gnss coordinates
        self.rover1_subscriber = self.create_subscription(UBXNavPVT, '/rover1/ubx_nav_pvt', self.rover1_callback, 10)
        self.rover2_subscriber = self.create_subscription(UBXNavPVT, '/rover2/ubx_nav_pvt', self.rover2_callback, 10)
        
        # Create client for path planning
        self.multipathplan_client = self.create_client(MultiPathPlan, 'multi_path_plan')
        
        # Create waypoint publisher
        self.waypoint_publisher = self.create_publisher(Float32MultiArray, 'waypoint', 10)
        
        # Create led publisher
        self.led_publisher = self.create_publisher(Float32MultiArray, 'led', 1)
        
        # Initialize global variables
        self.waypoint_msg = Float32MultiArray()  # Waypoint msg
        self.led_msg = Float32MultiArray()       # Led msg
        self.rover1_lat = None                   # Gnss coordinates
        self.rover1_lon = None
        self.rover2_lat = None
        self.rover2_lon = None
        self.loc = None                          # Current location
        self.points = {}                         # Input points
        self.waiting_for_path = False            # Waiting for path planning to generate path
        self.waypoints = []                      # Generated waypoints
        self.current_waypoint = 0                # Current waypoint index
        self.interval = 0.05                     # Timer callback interval (20 Hz)
        self.counter = 0                         # Control led flashing frequency
        
        # Get the filepath for points.txt
        points_filepath = os.path.join(get_package_share_directory('state_machine'), 'resource', 'points.txt')
        
        # Scan the points file
        with open(points_filepath, 'r') as f:
            lines = f.readlines()
            
            for line in lines:
                # Get rid of white spaces and split with spaces
                vals = line.strip().split()
                
                # Get the latitude, longitude, and point label (gnss, aruco, bottle, mallet, hammer)
                lat = float(vals[0])
                lon = float(vals[1])
                label = vals[2]
                
                # Store them in the points dictionary with (lat, lon) as key and label as value
                self.points[(lat, lon)] = label
        
        # Set led to red
        self.led_msg.data = [255.0, 0.0, 0.0]
        self.led_publisher.publish(self.led_msg)
        
        # Set state to planning
        self.state = "PLANNING"
        
        # Timer callback at 20 Hz
        self.timer = self.create_timer(self.interval, self.timer_callback)


    
    # Callback function for first antenna
    def rover1_callback(self, loc_msg):
        self.rover1_lat = loc_msg.lat * 1e-7
        self.rover1_lon = loc_msg.lon * 1e-7
        self.calculate_pos()
        
    # Callback function for second antenna
    def rover2_callback(self, loc_msg):
        self.rover2_lat = loc_msg.lat * 1e-7
        self.rover2_lon = loc_msg.lon * 1e-7
        self.calculate_pos()
        
    # Calculate the current location using the average of the two coordinates
    def calculate_pos(self):
        # Check if we've received at least one message from both antennas
        if None in [self.rover1_lat, self.rover1_lon, self.rover2_lat, self.rover2_lon]:
            return
            
        self.loc = [(self.rover1_lat + self.rover2_lat) / 2.0, (self.rover1_lon + self.rover2_lon) / 2.0]


    
    # Main loop
    def timer_callback(self):
        # TODO: If stop, Change state to manual
        
        if self.state == "PLANNING":
            self.planning()
            
        elif self.state == "NAV":
            self.nav()
            
        elif self.state == "ARUCO_NAV":
            self.aruco_nav()
            
        elif self.state == "OBJECT_NAV":
            self.object_nav()
            
        elif self.state == "FLASHING":
            self.flashing()
            
        elif self.state == "DANCE_OFF":
            self.dance_off()
            
        elif self.state == "MANUAL":
            self.manual()
    
    
    
    # Currently waiting for path planning to generate waypoints
    def planning(self):
        # If still waiting for location
        if self.loc == None:
            return
            
        # If waiting for path, skip
        if self.waiting_for_path:
            return
        
        # Send request to path planning
        req = MultiPathPlan.Request()
        
        # Starting point
        req.start = GeoPoint()
        req.start.latitude = self.loc[0]
        req.start.longitude = self.loc[1]
        
        # All points
        targets = []
        
        for key, value in self.points.items():
            point = GeoPoint()
            point.latitude = key[0]
            point.longitude = key[1]
            targets.append(point)
            
        req.targets = targets
        
        # Set waiting_for_path to true
        self.waiting_for_path = True
        
        # Async call
        future = self.multipathplan_client.call_async(req)
        
        # When we receive the result
        future.add_done_callback(self.path_callback)
        
        
        
    # Callback function that runs when we receive the result from path planning
    def path_callback(self, future):
        # Get the result
        result = future.result()
        
        # If for some reason the result is nonr or the generated path is empty
        if result == None or len(result.path) == 0:
            return
        
        # Get waypoints from generated path
        self.waypoints = [(waypoint.latitude, waypoint.longitude) for waypoint in result.path]
        
        # Publish the first waypoint
        self.waypoint_msg.data = [self.waypoints[0][0], self.waypoints[0][1]]
        self.waypoint_publisher.publish(self.waypoint_msg)
        
        # Set waiting_for_path to false and state to nav
        self.waiting_for_path = False
        self.state = "NAV"
        
        # TODO: Launch the autonomous navigation node in another terminal
    
    
    
    def nav(self):
        return
    
    
    def aruco_nav(self):
        return
    
    
    def object_nav(self):
        return
    
    
    def flashing(self):
        # TODO: If continue
        if False:
            # If reached the end of the waypoints list
            if self.current_waypoint == len(self.waypoints):
                # Change state to manual
                self.get_logger().info("Autonomous mission complete! Switching to manual mode.")
                self.state = "MANUAL"
            
            else:
                # Set led to red
                self.led_msg.data = [255.0, 0.0, 0.0]
                self.led_publisher.publish(self.led_msg)
                
                # Change state to nav
                self.state = "NAV"
            
            return
        
        # Increment counter
        self.counter += 1
        
        # Flash green at 2 Hz
        if self.counter % int(0.5 / self.interval) == 0:
            if self.led_msg.data != [0.0, 255.0, 0.0]:
                self.led_msg.data = [0.0, 255.0, 0.0]
            else:
                self.led_msg.data = [0.0, 0.0, 0.0]
                
            self.led_publisher.publish(self.led_msg)
    
    
    
    def dance_off(self):
        return
    
    
    def manual(self):
        # Set led to blue
        self.led_msg.data = [0.0, 0.0, 255.0] 
        self.led_publisher.publish(self.led_msg)
    
        
        
def main(args=None):
    rclpy.init(args=args)
    
    publisher = StateMachine()
    
    rclpy.spin(publisher)
    
    publisher.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
