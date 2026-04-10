#/usr/bin/env python3
 
import depthai as dai
import cv2
from numpy.matlib import empty
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
import math
import numpy as np
import time
import rclpy
from rclpy.node import Node
import cv_bridge
from std_msgs.msg import Float32MultiArray

from autonomous.aruco_class import ArucoClass
# from obstacle_avoidance_class import ObsAvoidaceClass
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
 
 
 
class AutonomousLogic(Node):
    def __init__(self):
        super().__init__('drive_logic')
 
        self.publisher_ = self.create_publisher(Float32MultiArray, '/swerve', 10)
 
        # TODO: Currently subscribing to GPS, change to RTK GNSS later
        self.localization_subscription = self.create_subscription(NavSatFix, 'fix', self.localization_callback, 10)
        
        self.imu_subscription = self.create_subscription(Imu, "imu/data", self.imu_callback, 10)
        
        self.CAMERA_WIDTH = 1280
        self.CAMERA_HEIGHT = 720
        
        self.pipeline = dai.Pipeline()
        cam = self.pipeline.create(dai.node.Camera).build()
        self.videoQueue = cam.requestOutput(
            (self.CAMERA_WIDTH, self.CAMERA_HEIGHT)
        ).createOutputQueue()
        self.pipeline.start()
 
        # 1. Initialize the heavy logic classes
        self.aruco_specialist = ArucoClass()
        # self.obstacle_specialist = ObsAvoidaceClass()
 
        # 2. Setup separate lanes (Callback Groups) for parallelism
        self.aruco_group = MutuallyExclusiveCallbackGroup()
        self.obstacle_group = MutuallyExclusiveCallbackGroup()
 
        # 3. Create Timers to run these in parallel
        self.create_timer(0.1, self.arucoLogic, callback_group=self.aruco_group)
        self.create_timer(0.05, self.obstacleAvoidanceLogic, callback_group=self.obstacle_group)
        self.create_timer(0.03, self.camera_callback)
        self.create_timer(0.05, self.autonomousCallback)
 
        self.localization_msg = None
        self.imu_msg = None
        self.odom_msg = None
        self.frame = None
 
        self.aruco_swerve = [0,0,0,0]
        self.obstacle_swerve = [0,0,0,0]
 
        self.zeroing_out_indicator = 0
 
 
    # These small 4 callbacsk are for updating the sensors messages
    def localization_callback(self, msg):
        self.localization_msg = msg
 
    def imu_callback(self, msg):
        self.imu_msg = msg
 
    def camera_callback(self):
        videoIn = self.videoQueue.tryGet()  # .get() stops the whole node until a frame arrives
 
        # This checks if it's not None AND tells PyCharm exactly what type it is
        if isinstance(videoIn, dai.ImgFrame):
            self.frame = videoIn.getCvFrame()
 
 
    # These 2 following methods are the main ones that will be callig the separate aruco and obstacle avoidance
    # classes. Each of them is running in a separate core of the CPU by using MutuallyExclusiveCallbackGroup
    # and MultiThreadedExecutor
    def arucoLogic(self):
 
        # Check if ANY of the required data is missing
        if self.frame is None or self.localization_msg is None or self.imu_msg is None:
            return
 
        current_time = self.get_clock().now()
        self.get_logger().info("Running arucoLogic")
        self.aruco_swerve = self.aruco_specialist.drive_logic(self.frame, self.localization_msg, self.imu_msg, current_time, self.zeroing_out_indicator)
 
        if self.zeroing_out_indicator < 1:
            self.zeroing_out_indicator += 1
 
 
    def obstacleAvoidanceLogic(self):
 
        # Check all required data for the obstacle logic
        if self.frame is None or self.localization_msg is None or self.imu_msg is None:
            return
 
        # Assuming get_swerve() is the method from obstacle avoidance that returns the swerve commands
        self.obstacle_swerve = [0,0,False]  #'''self.obstacle_specialist.get_swerve(self.frame, self.localization_msg, self.imu_msg, '''# self.odom_msg)
 
 
    def autonomousCallback(self):
 
        arucoSwerve = self.aruco_swerve
        # arucoSwerve format: [lin_y, lin_x, ang_pos, ang_neg]
 
        obsAvoidanceSwerve = self.obstacle_swerve
        # obsAvoidanceSwerve format: [linear_velocity, angular_velocity, obstacle_present_bool]
 
        sw_msg = Float32MultiArray()
 
        # FIX (point 1): the previous code published [arucoSwerve[0], arucoSwerve[1]],
        # which is [lin_y, lin_x] -- the angular components were silently dropped, so
        # the search-drive arc, the reorientation rotations, and the go_to_tag
        # correction never reached the rover. Consolidate the 4-element aruco swerve
        # into the [linear, angular] contract the receiver expects. Note that across
        # all branches of drive_logic, only one of {lin_x, lin_y} is non-zero at a
        # time, and only one of {ang_pos, ang_neg} is non-zero at a time, so summing
        # / subtracting them is safe.
        if obsAvoidanceSwerve[2] == False:
            aruco_linear = arucoSwerve[0] + arucoSwerve[1]      # lin_y + lin_x
            aruco_angular = arucoSwerve[2] - arucoSwerve[3]     # ang_pos - ang_neg
            sw_msg.data = [float(arucoSwerve[0]), float(arucoSwerve[1]), float(arucoSwerve[2]), float(arucoSwerve[3])]
        else:
            sw_msg.data = [float(obsAvoidanceSwerve[0]), float(obsAvoidanceSwerve[1])]
 
        self.get_logger().info("Publishing: " + str(sw_msg.data))
        self.publisher_.publish(sw_msg)
 
 
 
def main(args=None):
    rclpy.init(args=args)
 
    autonomous_node = AutonomousLogic()
 
    # Create the MultiThreadedExecutor
    # It will automatically detect them MutuallyExclusiveCallbackGroups and thread them
    executor = MultiThreadedExecutor()
    executor.add_node(autonomous_node)
 
    try:
        autonomous_node.get_logger().info("Starting Autonomous Node...")
        executor.spin()
    except KeyboardInterrupt:
        autonomous_node.get_logger().info("Shutting down...")
    finally:
        # FIX (point 14): the depthai pipeline was never being stopped, leaving the
        # camera resource held when the node exits. Stop it before destroying the
        # node. Wrapped in try/except so a failure here can't block destroy_node().
        try:
            autonomous_node.pipeline.stop()
        except Exception as e:
            autonomous_node.get_logger().warn(f"Error stopping depthai pipeline: {e}")
 
        autonomous_node.destroy_node()
        rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()
