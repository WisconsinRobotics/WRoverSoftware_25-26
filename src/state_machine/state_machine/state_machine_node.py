# === Import Packages ===
import os
import csv
import math
import json
import rclpy
import signal
import subprocess
from pathlib import Path
from typing import Tuple
from rclpy.node import Node
from std_msgs.msg import Bool
from enum import StrEnum, auto
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



# === Global Variables ===

# 20 Hz timer callback
TIMER_CALLBACK_INTERVAL = 0.05

# TODO: tune Dance off constants
STUCK_FRAMES_THRESHOLD = int(1 / TIMER_CALLBACK_INTERVAL) * 15      # 15 secs trigger threshold
STUCK_DISTANCE_THRESHOLD = 1                                        # 1 meter trigger threshold
DANCE_OFF_FRAMES_THRESHOLD = int(1 / TIMER_CALLBACK_INTERVAL) * 15  # 15 seconds dance off time

class ROVER_STATE(StrEnum):
    PLANNING = auto(),
    MANUAL = auto(),
    SPIRAL_PLANNING = auto(),
    NAV = auto(),
    ARUCO_NAV = auto(),
    OBJECT_NAV = auto(),
    FLASHING = auto()
    DANCE_OFF = auto(),

class ROVER_COMMAND(StrEnum):
    EMPTY = "",
    STOP = "stop",
    CONTINUE = "continue",
    SKIP = "skip"

class TARGET_LABEL(StrEnum):
    ARUCO1 = "aruco1",
    ARUCO2 = "aruco2",
    BOTTLE = "bottle",
    MALLET = "mallet",
    HAMMER = "hammer",
    GNSS = "gnss"



class StateMachineNode(Node):
    # === Initialization ===
    def __init__(self):
        super().__init__('state_machine')
        self.get_logger().info("Starting Autonomous Mission")
        
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

        # Create a direct swerve publisher (for DANCE_OFF)
        self.swerve_publisher = self.create_publisher(Float32MultiArray, '/swerve', 10)

        # Create led publisher
        self.led_publisher = self.create_publisher(Float32MultiArray, 'led', 1)
        
        # Initialize global variables
        self.xbox_controller = None                          # Xbox controller node
        self.nav_node = None                                 # Normal navigation node
        self.aruco_nav_node = None                           # Navigation with aruco node
        self.objecy_nav_node = None                          # Navigation with object node
        self.reached_signal = False                          # If aruco/object reached
        self.spiral = None                                   # Spiral GeoPath
        self.state_machine_controller = ROVER_COMMAND.EMPTY  # State machine controller command
        self.waypoint_msg = Float64MultiArray()              # Waypoint msg
        self.led_msg = Float32MultiArray()                   # Led msg
        self.spiral_msg = Float64MultiArray()                # Spiral msg
        self.rover1_lat = None                               # Gnss coordinates
        self.rover1_lon = None
        self.rover2_lat = None
        self.rover2_lon = None
        self.loc = None                                      # Current location
        self.points = {}                                     # Input points
        self.paths = []                                      # Each target has a path (list of lists)
        self.curr_target = 0                                 # Current target index
        self.curr_waypoint = 0                               # Current waypoint index
        self.interval = TIMER_CALLBACK_INTERVAL              # Timer callback interval (20 Hz)
        self.counter = 0                                     # Control led flashing frequency
        
        # Default points file is points.txt in /resource
        default_points_filepath = os.path.join(get_package_share_directory('state_machine'), 'resource', 'points.txt')

        # Option to configure different points file
        self.declare_parameter("points_file_path", default_points_filepath)
        self.points_filepath = self.get_parameter("points_file_path").get_parameter_value().string_value

        # Get the points filepath
        path = Path(self.points_filepath)

        # Scan the points file
        with open(self.points_filepath) as f:
            match path.suffix:
                case ".json":
                    d = json.load(f)
                    targets = d["targets"]

                    for target in targets:
                        lat, lon, label = target["lat"], target["lon"], target["label"]
                        self.points[(lat, lon)] = label

                case ".txt":
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
        self.state = ROVER_STATE.PLANNING

        # Stuck detection trackers
        self.last_rover_position = None
        self.stuck_frames = 0

        # DANCE_OFF fix trackers
        self.last_state = None
        self.dance_off_frames = 0

        # Timer callback at 20 Hz
        self.timer = self.create_timer(self.interval, self.timer_callback)
        
        

    # === Callback Functions ===

    # Callback function for path
    def path_callback(self, future):
        result = future.result()
        self.get_logger().info("Path planning result received")
        
        # Get paths
        for i in range(len(result.path)):
            path = []
            
            for j in range(len(result.path[i].poses)):
                # Get the latitude and longitude
                lat = result.path[i].poses[j].pose.position.latitude
                lon = result.path[i].poses[j].pose.position.longitude

                # Remove duplicate targets at the start of a path
                if i == 0 or j > 0:
                    path.append((lat, lon))

                # Replace the inaccurate target at the end of each path with the actual target
                if j == len(result.path[i].poses) - 1:
                    target = (lat, lon)
                    target = min(self.points.keys(), key=lambda k: self.haversine(k[0], k[1], target[0], target[1])) 
                    path.append(target)
                    
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
        # if None in [self.rover1_lat, self.rover1_lon, self.rover2_lat, self.rover2_lon]:
        #    return
            
        # self.loc = [(self.rover1_lat + self.rover2_lat) / 2.0, (self.rover1_lon + self.rover2_lon) / 2.0]
        self.loc = [self.rover1_lat, self.rover1_lon]


    
    # === Main Loop ===
    def timer_callback(self):
        # If stop, Change state to manual
        if self.state_machine_controller == ROVER_COMMAND.STOP:
            self.state_machine_controller = ROVER_COMMAND.EMPTY
            self.get_logger().info("Autonomous operation stopped, switching to manual mode")
            self.state = ROVER_STATE.MANUAL
        
        # Track dance off
        self.dance_off_tracking()

        # If we are in navigation and got stuck for a long time, transition to dance off state
        if self.stuck_frames >= STUCK_FRAMES_THRESHOLD and self.state in [ROVER_STATE.NAV, ROVER_STATE.ARUCO_NAV, ROVER_STATE.OBJECT_NAV]:
            # Stop nav node
            if self.state == ROVER_STATE.NAV:
                self.stop_node(self.nav_node)

            if self.state == ROVER_STATE.ARUCO_NAV:
                self.stop_node(self.aruco_nav_node)

            if self.state == ROVER_STATE.OBJECT_NAV:
                self.stop_node(self.object_nav_node)

            # Set the dance off constants
            self.last_state = self.state
            self.state = ROVER_STATE.DANCE_OFF
            self.stuck_frames = 0
            self.last_rover_position = self.loc
            self.dance_off_frames = 0
            self.get_logger().info("Rover stuck, starting dance off mode")

        match self.state:
            case ROVER_STATE.PLANNING:
                self.planning()

            case ROVER_STATE.SPIRAL_PLANNING:
                self.spiral_planning()

            case ROVER_STATE.NAV:
                self.nav()

            case ROVER_STATE.ARUCO_NAV:
                self.aruco_nav()

            case ROVER_STATE.OBJECT_NAV:
                self.object_nav()

            case ROVER_STATE.FLASHING:
                self.flashing()

            case ROVER_STATE.DANCE_OFF:
                self.dance_off()

            case ROVER_STATE.MANUAL:
                self.manual() 
    


    # === States ===

    # Waiting for path planning to generate path
    def planning(self):
        if len(self.paths) != 0:
            # Set state to spiral planning
            self.get_logger().info("Waiting for spiral search path planning result")
            self.state = ROVER_STATE.SPIRAL_PLANNING
    
    # Waiting for spiral search path planning to generate path
    def spiral_planning(self):
        # Iterated through all the targets
        if self.curr_target == len(self.paths):
            # Set current target back to 0 and set current waypoint to 1
            self.curr_target = 0
            self.curr_waypoint = 1
            
            # Launch the nav node
            self.nav_node = subprocess.Popen(["ros2", "run", "navigation", "nav"])
            
            # Export paths and set state to nav
            self.export_paths_to_csv()
            self.get_logger().info("Starting normal navigation")
            self.state = ROVER_STATE.NAV
            
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
                if self.points[target] == TARGET_LABEL.ARUCO1:
                    self.spiral_msg.data = [target[0], target[1], self.paths[self.curr_target][-2][0], self.paths[self.curr_target][-2][1], 5.0, 10.0]
                    self.spiral_publisher.publish(self.spiral_msg)
                    
                # aruco2
                elif self.points[target] == TARGET_LABEL.ARUCO2:
                    self.spiral_msg.data = [target[0], target[1], self.paths[self.curr_target][-2][0], self.paths[self.curr_target][-2][1], 10.0, 20.0]
                    self.spiral_publisher.publish(self.spiral_msg)
                    
                # mallet
                elif self.points[target] == TARGET_LABEL.MALLET:
                    self.spiral_msg.data = [target[0], target[1], self.paths[self.curr_target][-2][0], self.paths[self.curr_target][-2][1], 0.0, 3.0]
                    self.spiral_publisher.publish(self.spiral_msg)
                    
                # hammer
                elif self.points[target] == TARGET_LABEL.HAMMER:
                    self.spiral_msg.data = [target[0], target[1], self.paths[self.curr_target][-2][0], self.paths[self.curr_target][-2][1], 0.0, 3.0]
                    self.spiral_publisher.publish(self.spiral_msg)
                    
                # bottle
                elif self.points[target] == TARGET_LABEL.BOTTLE:
                    self.spiral_msg.data = [target[0], target[1], self.paths[self.curr_target][-2][0], self.paths[self.curr_target][-2][1], 0.0, 10.0]
                    self.spiral_publisher.publish(self.spiral_msg)
                
                # Skip spiral if gnss
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
            self.stop_node(self.nav_node)
            
            # Change state to flashing
            self.get_logger().info("Gnss point reached, switching to flashing mode")
            self.state = ROVER_STATE.FLASHING
            
            return
            
        # Match with the closest target
        target = self.paths[self.curr_target][self.curr_waypoint]
        target = min(self.points.keys(), key=lambda k: self.haversine(k[0], k[1], target[0], target[1])) 
        
        # If within 20m of aruco/object
        if self.haversine(self.loc[0], self.loc[1], target[0], target[1]) < 20 and self.points[target] != "gnss":
            # Stop nav node
            if self.nav_node != None:
                self.stop_node(self.nav_node)
                
            # Aruco
            if self.points[target] == TARGET_LABEL.ARUCO1 or self.points[target] == TARGET_LABEL.ARUCO2:
                # Start aruco nav node
                self.aruco_nav_node = subprocess.Popen(["ros2", "run", "navigation", "detector"])
                
                # Change state to aruco nav
                self.get_logger().info("Approaching aruco point, switching to aruco navigation")
                self.state = ROVER_STATE.ARUCO_NAV
                
                return
            
            # Mallet
            if self.points[target] == TARGET_LABEL.MALLET:
                # Start object nav node
                self.object_nav_node = subprocess.Popen(["ros2", "run", "navigation", "object_detection", "--ros-args -p model:='mallet'"])
                
                # Change state to object nav
                self.get_logger().info("Approaching mallet point, switching to object navigation")
                self.state = ROVER_STATE.OBJECT_NAV
                
                return
            
            # Hammer
            if self.points[target] == TARGET_LABEL.HAMMER:
                # Start object nav node
                self.object_nav_node = subprocess.Popen(["ros2", "run", "navigation", "object_detection", "--ros-args -p model:='hammer'"])
                
                # Change state to object nav
                self.get_logger().info("Approaching hammer point, switching to object navigation")
                self.state = ROVER_STATE.OBJECT_NAV
                
                return
            
            # Bottle
            if self.points[target] == TARGET_LABEL.BOTTLE:
                # Start object nav node
                self.object_nav_node = subprocess.Popen(["ros2", "run", "navigation", "object_detection", "--ros-args -p model:='bottle'"])
                
                # Change state to object nav
                self.get_logger().info("Approaching bottle point, switching to object navigation")
                self.state = ROVER_STATE.OBJECT_NAV
                
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
            self.stop_node(self.aruco_nav_node)
            
            # Change state to nav
            self.get_logger().info("Traversed entire search spiral but no aruco tag found, switching to normal navigation")
            self.state = ROVER_STATE.NAV
            
            return
            
        # If reached aruco
        if self.reached_signal:
            self.reached_signal = False
            
            # Increment current target and set current waypoint to 0
            self.curr_target += 1
            self.curr_waypoint = 0
            
            # Stop aruco nav node
            self.stop_node(self.aruco_nav_node)
            
            # Change state to flashing
            self.get_logger().info("Aruco tag reached, switching to flashing mode")
            self.state = ROVER_STATE.FLASHING
            
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
            self.stop_node(self.object_nav_node)
            
            # Change state to nav
            self.get_logger().info("Traversed entire search spiral but no object found, switching to normal navigation")
            self.state = ROVER_STATE.NAV
            
            return
            
        # If reached object
        if self.reached_signal:
            self.reached_signal = False
            
            # Increment current target and set current waypoint to 0
            self.curr_target += 1
            self.curr_waypoint = 0
            
            # Stop object nav node
            self.stop_node(self.object_nav_node)
            
            # Change state to flashing
            self.get_logger().info("Object reached, switching to flashing mode")
            self.state = ROVER_STATE.FLASHING
            
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
            self.stop_node(self.nav_node)
            
        if self.aruco_nav_node != None:
            self.stop_node(self.aruco_nav_node)
            
        if self.object_nav_node != None:
            self.stop_node(self.object_nav_node)
            
        # If continue
        if self.state_machine_controller == ROVER_COMMAND.CONTINUE:
            self.state_machine_controller = ROVER_COMMAND.EMPTY
            
            # If reached the end of the targets
            if self.curr_target == len(self.paths):
                # Change state to manual
                self.get_logger().info("Autonomous Mission complete!")
                self.get_logger().info("Switching to manual mode")
                self.state = ROVER_STATE.MANUAL
            
            else:
                # Set led to red
                self.led_msg.data = [255.0, 0.0, 0.0]
                self.led_publisher.publish(self.led_msg)
                
                # Increment target
                self.curr_target += 1
                
                # Start nav node
                self.nav_node = subprocess.Popen(["ros2", "run", "navigation", "nav"])
                
                # Change state to nav
                self.get_logger().info("Continuing normal navigation")
                self.state = ROVER_STATE.NAV
            
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
        # If we were in dance off more than threshold, return back to state prior to dance off
        if self.dance_off_frames >= DANCE_OFF_FRAMES_THRESHOLD:
            # Start nav node
            self.nav_node = subprocess.Popen(["ros2", "run", "navigation", "nav"])
            
            # Change state to nav
            self.get_logger().info("Exited dance off, continuing normal navigation")
            self.state = ROVER_STATE.NAV
            
            # Reset dance off variables
            self.last_state = None
            self.stuck_frames = 0
            self.last_rover_position = self.loc
            self.dance_off_frames = 0

            return
        
        # Go straight back
        msg = Float32MultiArray()
        msg.data = [-1.0, -1.0, -1.0, -1.0]
        self.swerve_publisher.publish(msg)

        self.dance_off_frames += 1
    
    def manual(self):
        # Set led to blue
        self.led_msg.data = [0.0, 0.0, 255.0] 
        self.led_publisher.publish(self.led_msg)
        
        # Stop any nav node
        if self.nav_node != None:
            self.stop_node(self.nav_node)
            
        if self.aruco_nav_node != None:
            self.stop_node(self.aruco_nav_node)
            
        if self.object_nav_node != None:
            self.stop_node(self.object_nav_node)
        
        # Start Xbox controller node
        if self.xbox_controller == None:
            self.xbox_controller = subprocess.Popen(["ros2", "run", "wr_xbox_controller", "drive_controller"])
        
        # If continue
        if self.state_machine_controller == ROVER_COMMAND.CONTINUE:
            self.state_machine_controller = ROVER_COMMAND.EMPTY
            
            # Stop Xbox controller node
            if self.xbox_controller != None:
                self.stop_node(self.xbox_controller)
                
            # Start nav node
            self.nav_node = subprocess.Popen(["ros2", "run", "navigation", "nav"])
            
            # Change state to nav
            self.get_logger().info("Continuing normal navigation")
            self.state = ROVER_STATE.NAV

        # If skip
        if self.state_machine_controlelr ==  ROVER_COMMAND.SKIP:
            self.state_machine_controller = ROVER_COMMAND.EMPTY

            # Stop Xbox controller node
            if self.xbox_controller != None:
                self.stop_node(self.xbox_controller)

            # Increment current target and set current waypoint to 0
            self.curr_target += 1
            self.curr_waypoint = 0

            # Start nav node
            self.nav_node = subprocess.Popen(["ros2", "run", "navigation", "nav"])
            
            # Change state to nav
            self.get_logger().info("Skipped current target, continuing normal navigation to next target")
            self.state = ROVER_STATE.NAV



    # === Helper Functions ===

    # Function to track dance off
    def dance_off_tracking(self):
        # If in dance off, skip directly to dance off handler
        if self.state == ROVER_STATE.DANCE_OFF:
            return
        
        # If not in dance off, reset dance off fix trackers
        self.dance_off_frames = 0
        self.last_state = None

        if not self.last_rover_position:
            self.last_rover_position = self.loc
            self.stuck_frames = 0
            return
        
        # If we are not in navigation, don't track stuck detection
        if self.state not in [ROVER_STATE.NAV, ROVER_STATE.ARUCO_NAV, ROVER_STATE.OBJECT_NAV]:
            self.stuck_frames = 0
            self.last_rover_position = self.loc
            return
        
        # Track for how long rover hasn't changed its position
        last_lat, last_lon = self.last_rover_position
        current_lat, current_lon = self.loc
        if self.haversine(last_lat, last_lon, current_lat, current_lon) < STUCK_DISTANCE_THRESHOLD:
            self.stuck_frames += 1
        else:
            self.stuck_frames = 0
            self.last_rover_position = self.loc

    # Helper function to stop swerve and stop nodes
    def stop_node(self, node):
        msg = Float32MultiArray()
        msg.data = [0.0, 0.0, -1.0, -1.0]
        self.swerve_publisher.publish(msg)

        node.send_signal(signal.SIGINT)
        node = None
        
    # Export the entire path to a csv file
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
        
    # Calculate the distance between 2 gnss coordinates
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
    
    # Calculate bearing angle in radians from current to target
    def bearing_between_points(self, current: Tuple[float], target: Tuple[float]) -> float:
        lat1 = math.radians(current[0])
        lon1 = math.radians(current[1])

        lat2 = math.radians(target[0])
        lon2 = math.radians(target[1])

        dlon = lon2 - lon1

        y = math.sin(dlon) * math.cos(lat2)
        x = (
            math.cos(lat1) * math.sin(lat2)
            - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        )

        return math.atan2(y, x)
        


# === Main ===
def main(args=None):
    rclpy.init(args=args)
    
    state_machine = StateMachineNode()
    
    rclpy.spin(state_machine)
    
    state_machine.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
