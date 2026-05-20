import os
import csv
import math
import rclpy
import signal
import subprocess
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import String
from geographic_msgs.msg import GeoPath
from geographic_msgs.msg import GeoPose
from geographic_msgs.msg import GeoPoint
from ublox_ubx_msgs.msg import UBXNavPVT
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64MultiArray
from wr_interfaces.srv import MultiPathPlan
from geographic_msgs.msg import GeoPoseStamped
from ament_index_python.packages import get_package_share_directory



class StateMachine(Node):
    def __init__(self):
        # Initialization
        super().__init__('state_machine')
        self.get_logger().info("Starting Autonomous Mission")
        
        # TODO: Start swerve nodes
        # self.swerve_control = subprocess.Popen(["ros2", "run", "wr_swerve_control", "swerve_control"])
        # self.swerve_motor = subprocess.Popen(["ros2", "run", "wr_swerve_motor", "swerve_motor"])
        
        # Create publisher and subscriber for spiral path generation node
        self.spiral_publisher = self.create_publisher(Float64MultiArray, 'spiral_request', 10)
        self.spiral_subscriber = self.create_subscription(GeoPath, 'spiral_path', self.spiral_callback, 10)
        
        # Create subscribers for gnss coordinates
        self.rover1_subscriber = self.create_subscription(UBXNavPVT, 'rover1/ubx_nav_pvt', self.rover1_callback, 10)
        self.rover2_subscriber = self.create_subscription(UBXNavPVT, 'rover2/ubx_nav_pvt', self.rover2_callback, 10)
        
        # Create subscriber to state machine controller
        self.state_machine_controller_subscriber = self.create_subscription(String, '/state_machine_controller', self.state_machine_controller_callback, 10)
        
        # Create subscriber for aruco/object reached
        self.reached_signal = self.create_subscription(Bool, "reached_signal", self.reached_signal_callback, 10)
        
        # Create client for path planning
        self.multipathplan_client = self.create_client(MultiPathPlan, 'multi_path_plan')
        
        # Create waypoint publisher
        self.waypoint_publisher = self.create_publisher(Float64MultiArray, 'waypoint', 10)
        
        # Create led publisher
        self.led_publisher = self.create_publisher(Float32MultiArray, 'led', 1)
        
        # Initialize global variables
        self.xbox_controller = None              # Xbox controller node
        self.nav_node = None                     # Normal navigation node
        self.aruco_nav_node = None               # Navigation with aruco node
        self.objecy_nav_node = None              # Navigation with object node
        self.reached_signal = False              # If aruco/object reached
        self.spiral = None                       # Spiral GeoPath
        self.state_machine_controller = ""       # State machine controller command
        self.waypoint_msg = Float64MultiArray()  # Waypoint msg
        self.led_msg = Float32MultiArray()       # Led msg
        self.spiral_msg = Float64MultiArray()    # Spiral msg
        self.rover1_lat = None                   # Gnss coordinates
        self.rover1_lon = None
        self.rover2_lat = None
        self.rover2_lon = None
        self.loc = None                          # Current location
        self.points = {}                         # Input points
        self.paths = []                          # Each target has a path (list of lists)
        self.curr_target = 0                     # Current target index
        self.curr_waypoint = 0                   # Current waypoint index
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
                
                # Get the latitude, longitude, and point label (gnss, aruco1, aruco2, mallet, hammer, bottle)
                lat = float(vals[0])
                lon = float(vals[1])
                label = vals[2]
                
                # Store them in the points dictionary with (lat, lon) as key and label as value
                self.points[(lat, lon)] = label
                
        # Set led to red
        self.get_logger().info("Successfully read target points")
        self.led_msg.data = [255.0, 0.0, 0.0]
        self.led_publisher.publish(self.led_msg)
        
        # Path planning request
        req = MultiPathPlan.Request()
        
        # Starting point
        req.start = GeoPoint()
        # TODO
        # req.start.latitude = self.loc[0]
        # req.start.longitude = self.loc[1]
        req.start.latitude = 43.07061877920905
        req.start.longitude = -89.40977230012103
        
        # Target points
        targets = []
        for key, value in self.points.items():
            point = GeoPoint()
            point.latitude = key[0]
            point.longitude = key[1]
            targets.append(point)
        req.targets = targets
        
        # Asynchronous service call to path planning
        self.multipathplan_client.wait_for_service(timeout_sec=5.0)
        future = self.multipathplan_client.call_async(req)
        future.add_done_callback(self.path_callback)
        
        # Set state to planning
        self.get_logger().info("Waiting for path planning result")
        self.state = "PLANNING"
        
        # Timer callback at 20 Hz
        self.timer = self.create_timer(self.interval, self.timer_callback)
        
        
        
    # Callback function for path
    def path_callback(self, future):
        result = future.result()
        self.get_logger().info("Path planning result received")
        
        # Get paths
        for i in range(len(result.path)):
            path = []
            
            for j in range(len(result.path[i].poses)):
                # Remove duplicate targets at the start of a path
                if i == 0 or j > 0:
                    path.append((result.path[i].poses[j].pose.position.latitude, result.path[i].poses[j].pose.position.longitude))
                    
            self.paths.append(path)
        
    # Callback function for spiral path
    def spiral_callback(self, path):
        self.spiral = path
        
    # Callback function for state machine controller
    def state_machine_controller_callback(self, cmd):
        self.state_machine_controller = cmd.data
        
    # Callback function for aruco/object reached
    def reached_signal_callback(self, sig):
        self.reached_signal = sig.data
    
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
        # If stop, Change state to manual
        if self.state_machine_controller == "stop":
            self.state_machine_controller = ""
            self.get_logger().info("Autonomous operation stopped, switching to manual mode")
            self.state = "MANUAL"
        
        if self.state == "PLANNING":
            self.planning()
            
        elif self.state == "SPIRAL_PLANNING":
            self.spiral_planning()
            
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
    
    # Waiting for path planning to generate path
    def planning(self):
        if len(self.paths) != 0:
            # Set state to spiral planning
            self.get_logger().info("Waiting for spiral search path planning result")
            self.state = "SPIRAL_PLANNING"
    
    # Waiting for spiral search path planning to generate path
    def spiral_planning(self):
        # Iterated through all the targets
        if self.curr_target == len(self.paths):
            # Set current target back to 0 and set current waypoint to 1
            self.curr_target = 0
            self.curr_waypoint = 1
            
            # TODO: Launch the nav node
            self.nav_node = subprocess.Popen(["ros2", "run", "nav", "nav"])
            
            # Export paths and set state to nav
            self.export_paths_to_csv()
            self.get_logger().info("Starting normal navigation")
            self.state = "NAV"
            
            # Publish the first waypoint
            self.waypoint_msg.data = [self.paths[self.curr_target][self.curr_waypoint][0], self.paths[self.curr_target][self.curr_waypoint][1]]
            self.waypoint_publisher.publish(self.waypoint_msg)
            self.get_logger().info(f"Published waypoint [{self.waypoint_msg.data[0]}, {self.waypoint_msg.data[1]}]")
            
            # Increment current waypoint (assuming each path has at least 3 total points)
            self.curr_waypoint += 1
            
            return
            
        # Get spiral search paths for aruco1, aruco2, mallet, hammer, and bottle
        if self.spiral == None:   
            # Match with the closest target
            target = self.paths[self.curr_target][-1]
            target = min(self.points.keys(), key=lambda k: self.haversine(k[0], k[1], target[0], target[1]))
            
            # If we didn't publish yet
            if len(self.spiral_msg.data) == 0:
                # aruco1
                if self.points[target] == 'aruco1':
                    self.spiral_msg.data = [target[0], target[1], self.paths[self.curr_target][-2][0], self.paths[self.curr_target][-2][1], 5.0, 10.0]
                    self.spiral_publisher.publish(self.spiral_msg)
                    
                # aruco2
                elif self.points[target] == 'aruco2':
                    self.spiral_msg.data = [target[0], target[1], self.paths[self.curr_target][-2][0], self.paths[self.curr_target][-2][1], 10.0, 20.0]
                    self.spiral_publisher.publish(self.spiral_msg)
                    
                # mallet
                elif self.points[target] == 'mallet':
                    self.spiral_msg.data = [target[0], target[1], self.paths[self.curr_target][-2][0], self.paths[self.curr_target][-2][1], 0.0, 3.0]
                    self.spiral_publisher.publish(self.spiral_msg)
                    
                # hammer
                elif self.points[target] == 'hammer':
                    self.spiral_msg.data = [target[0], target[1], self.paths[self.curr_target][-2][0], self.paths[self.curr_target][-2][1], 0.0, 3.0]
                    self.spiral_publisher.publish(self.spiral_msg)
                    
                # bottle
                elif self.points[target] == 'bottle':
                    self.spiral_msg.data = [target[0], target[1], self.paths[self.curr_target][-2][0], self.paths[self.curr_target][-2][1], 0.0, 10.0]
                    self.spiral_publisher.publish(self.spiral_msg)
                
                # Increment current target
                else:
                    self.curr_target += 1
                    
        # Spiral path received 
        else:
            # Append path to the end of the current target path
            for pose in self.spiral.poses:
                self.paths[self.curr_target].append((pose.pose.position.latitude, pose.pose.position.longitude))
                
            # Reset spiral msg data to empty
            self.spiral_msg.data = []
                
            # Reset spiral path to none
            self.spiral = None
                
            # Increment current target
            self.curr_target += 1
            
    def nav(self):
        # If reached end of path (this would only happen if reached gnss target since added spiral path)
        if self.curr_waypoint == len(self.paths[self.curr_target]):
            # Increment current target and set current waypoint to 0
            self.curr_target += 1
            self.curr_waypoint = 0
            
            # Stop nav node
            self.nav_node.send_signal(signal.SIGINT)
            self.nav_node = None
            
            # Change state to flashing
            self.get_logger().info("Gnss point reached, switching to flashing mode")
            self.state = "FLASHING"
            
            return
            
        # Match with the closest target
        target = self.paths[self.curr_target][self.curr_waypoint]
        target = min(self.points.keys(), key=lambda k: self.haversine(k[0], k[1], target[0], target[1])) 
        
        # If within 20m of aruco/object (assuming aruco/object points are at least 40m apart)
        if self.haversine(self.loc[0], self.loc[1], target[0], target[1]) < 20 and self.points[target] != "gnss":
            # Stop nav node
            if self.nav_node != None:
                self.nav_node.send_signal(signal.SIGINT)
                self.nav_node = None
                
            # Aruco
            if self.points[target] == "aruco1" or self.points[target] == "aruco2":
                # TODO: Start aruco nav node
                self.aruco_nav_node = subprocess.Popen(["ros2", "run", "nav", "aruco"])
                
                # Change state to aruco nav
                self.get_logger().info("Approaching aruco point, switching to aruco navigation")
                self.state = "ARUCO_NAV"
                
                return
            
            # Mallet
            if self.points[target] == "mallet":
                # TODO: Start object nav node
                self.object_nav_node = subprocess.Popen(["ros2", "run", "nav", "object", "mallet"])
                
                # Change state to object nav
                self.get_logger().info("Approaching mallet point, switching to object navigation")
                self.state = "OBJECT_NAV"
                
                return
            
            # Hammer
            if self.points[target] == "hammer":
                # TODO: Start object nav node
                self.object_nav_node = subprocess.Popen(["ros2", "run", "nav", "object", "hammer"])
                
                # Change state to object nav
                self.get_logger().info("Approaching hammer point, switching to object navigation")
                self.state = "OBJECT_NAV"
                
                return
            
            # Bottle
            if self.points[target] == "bottle":
                # TODO: Start object nav node
                self.object_nav_node = subprocess.Popen(["ros2", "run", "nav", "object", "bottle"])
                
                # Change state to object nav
                self.get_logger().info("Approaching bottle point, switching to object navigation")
                self.state = "OBJECT_NAV"
                
                return
                
        # If distance between current location and waypoint is within 0.6m
        if self.haversine(self.loc[0], self.loc[1], self.paths[self.curr_target][self.curr_waypoint][0], self.paths[self.curr_target][self.curr_waypoint][1]) < 0.6:
            # Increment current waypoint
            self.curr_waypoint += 1
            
            # Publish the next waypoint
            self.waypoint_msg.data = [self.paths[self.curr_target][self.curr_waypoint][0], self.paths[self.curr_target][self.curr_waypoint][1]]
            self.waypoint_publisher.publish(self.waypoint_msg)
            self.get_logger().info(f"Published waypoint [{self.waypoint_msg.data[0]}, {self.waypoint_msg.data[1]}]")
     
    def aruco_nav(self):
        # If reached end of path but no aruco found
        if self.curr_waypoint == len(self.paths[self.curr_target]) and self.reached_signal == False:
            # Increment current target and set current waypoint to 0
            self.curr_target += 1
            self.curr_waypoint = 0
            
            # Stop aruco nav node
            self.aruco_nav_node.send_signal(signal.SIGINT)
            self.aruco_nav_node = None
            
            # Change state to nav
            self.get_logger().info("Traversed entire search spiral but no aruco tag found, switching to normal navigation")
            self.state = "NAV"
            
            return
            
        # If reached aruco
        if self.reached_signal:
            self.reached_signal = False
            
            # Increment current target and set current waypoint to 0
            self.curr_target += 1
            self.curr_waypoint = 0
            
            # Stop aruco nav node
            self.aruco_nav_node.send_signal(signal.SIGINT)
            self.aruco_nav_node = None
            
            # Change state to flashing
            self.get_logger().info("Aruco tag reached, switching to flashing mode")
            self.state = "FLASHING"
            
            return
            
        # If distance between current location and waypoint is within 0.6m
        if self.haversine(self.loc[0], self.loc[1], self.paths[self.curr_target][self.curr_waypoint][0], self.paths[self.curr_target][self.curr_waypoint][1]) < 0.6:
            # Increment current waypoint
            self.curr_waypoint += 1
            
            # Publish the next waypoint
            self.waypoint_msg.data = [self.paths[self.curr_target][self.curr_waypoint][0], self.paths[self.curr_target][self.curr_waypoint][1]]
            self.waypoint_publisher.publish(self.waypoint_msg)
            self.get_logger().info(f"Published waypoint [{self.waypoint_msg.data[0]}, {self.waypoint_msg.data[1]}]")
    
    def object_nav(self):
        # If reached end of path but no object found
        if self.curr_waypoint == len(self.paths[self.curr_target]) and self.reached_signal == False:
            # Increment current target and set current waypoint to 0
            self.curr_target += 1
            self.curr_waypoint = 0
            
            # Stop object nav node
            self.object_nav_node.send_signal(signal.SIGINT)
            self.object_nav_node = None
            
            # Change state to nav
            self.get_logger().info("Traversed entire search spiral but no object found, switching to normal navigation")
            self.state = "NAV"
            
            return
            
        # If reached object
        if self.reached_signal:
            self.reached_signal = False
            
            # Increment current target and set current waypoint to 0
            self.curr_target += 1
            self.curr_waypoint = 0
            
            # Stop object nav node
            self.object_nav_node.send_signal(signal.SIGINT)
            self.object_nav_node = None
            
            # Change state to flashing
            self.get_logger().info("Object reached, switching to flashing mode")
            self.state = "FLASHING"
            
            return
            
        # If distance between current location and waypoint is within 0.6m
        if self.haversine(self.loc[0], self.loc[1], self.paths[self.curr_target][self.curr_waypoint][0], self.paths[self.curr_target][self.curr_waypoint][1]) < 0.6:
            # Increment current waypoint
            self.curr_waypoint += 1
            
            # Publish the next waypoint
            self.waypoint_msg.data = [self.paths[self.curr_target][self.curr_waypoint][0], self.paths[self.curr_target][self.curr_waypoint][1]]
            self.waypoint_publisher.publish(self.waypoint_msg)
            self.get_logger().info(f"Published waypoint [{self.waypoint_msg.data[0]}, {self.waypoint_msg.data[1]}]")
    
    def flashing(self):
        # Stop any nav node
        if self.nav_node != None:
            self.nav_node.send_signal(signal.SIGINT)
            self.nav_node = None
            
        if self.aruco_nav_node != None:
            self.aruco_nav_node.send_signal(signal.SIGINT)
            self.aruco_nav_node = None
            
        if self.object_nav_node != None:
            self.object_nav_node.send_signal(signal.SIGINT)
            self.object_nav_node = None
            
        # If continue
        if self.state_machine_controller == "continue":
            self.state_machine_controller = ""
            
            # If reached the end of the targets
            if self.curr_target == len(self.paths):
                # Change state to manual
                self.get_logger().info("Autonomous Mission complete!")
                self.get_logger().info("Switching to manual mode")
                self.state = "MANUAL"
            
            else:
                # Set led to red
                self.led_msg.data = [255.0, 0.0, 0.0]
                self.led_publisher.publish(self.led_msg)
                
                # Increment target
                self.curr_target += 1
                
                # TODO: Start nav node
                self.nav_node = subprocess.Popen(["ros2", "run", "nav", "nav"])
                
                # Change state to nav
                self.get_logger().info("Continuing normal navigation")
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
        
        # Stop any nav node
        if self.nav_node != None:
            self.nav_node.send_signal(signal.SIGINT)
            self.nav_node = None
            
        if self.aruco_nav_node != None:
            self.aruco_nav_node.send_signal(signal.SIGINT)
            self.aruco_nav_node = None
            
        if self.object_nav_node != None:
            self.object_nav_node.send_signal(signal.SIGINT)
            self.object_nav_node = None
        
        # Start Xbox controller node
        if self.xbox_controller == None:
            self.xbox_controller = subprocess.Popen(["ros2", "run", "wr_xbox_controller", "drive_controller"])
        
        # If continue
        if self.state_machine_controller == "continue":
            self.state_machine_controller = ""
            
            # Stop Xbox controller node
            if self.xbox_controller != None:
                self.xbox_controller.send_signal(signal.SIGINT)
                self.xbox_controller = None
                
            # TODO: Start nav node
            self.nav_node = subprocess.Popen(["ros2", "run", "nav", "nav"])
            
            # Change state to nav
            self.get_logger().info("Continuing normal navigation")
            self.state = "NAV"
        
        
        
    def export_paths_to_csv(self):
        # Define file path to write to
        path_filepath = os.path.expanduser('~/path.csv')

        with open(path_filepath, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["lat", "lon", "label"])

            for path in self.paths:
                # Write all waypoints
                for p in path[:-1]:
                    writer.writerow([p[0], p[1], "waypoint"])

                # Match and write target
                target = path[-1]
                target = min(self.points.keys(), key=lambda k: self.haversine(k[0], k[1], target[0], target[1]))
                writer.writerow([path[-1][0], path[-1][1], self.points[target]])

        self.get_logger().info(f"Spiral search paths received, exported to {path_filepath}")
        
    # TODO: check if this is correct
    def haversine(self, lat1, lon1, lat2, lon2):
        R = 6371000.0

        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)

        a = (math.sin(dphi / 2) ** 2 +
             math.cos(phi1) * math.cos(phi2) *
             math.sin(dlambda / 2) ** 2)

        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        return R * c
        
    
        
def main(args=None):
    rclpy.init(args=args)
    
    state_machine = StateMachine()
    
    rclpy.spin(state_machine)
    
    state_machine.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
