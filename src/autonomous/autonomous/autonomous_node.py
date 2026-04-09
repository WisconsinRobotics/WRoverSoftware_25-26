#!/usr/bin/env python3

import depthai as dai
from sensor_msgs.msg import Image
import cv2
from numpy.matlib import empty
from sensor_msgs.msg import Image
import math
import numpy as np
import time
import rclpy
from rclpy.node import Node
import cv_bridge
from std_msgs.msg import Float32MultiArray

from aruco_class import ArucoClass
from obstacle_avoidance_class import ObsAvoidaceClass
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor



class AutonomousLogic(Node):
    def __init__(self):
        super().__init__('drive_logic')

        self.publisher_ = self.create_publisher(Float32MultiArray, '/swerve', 10)

        self.localization_subscription = self.create_subscription(Float32MultiArray, 'localization_info',
                                                                  self.localization_callback, 10)
        self.imu_subscription = self.create_subscription(Float32MultiArray, "imu_info", self.imu_callback, 10)
        self.odom_subscription = self.create_subscription(Float32MultiArray, "odom_info", self.odom_callback, 10)

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
        self.obstacle_specialist = ObsAvoidaceClass()

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


    # These small 4 callbacsk are for updating the sensors messages
    def localization_callback(self, msg):
        self.localization_msg = msg

    def imu_callback(self, msg):
        self.imu_msg = msg

    def odom_callback(self, msg):
        self.odom_msg = msg

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

        self.aruco_swerve = self.aruco_specialist.drive_logic(self.frame, self.localization_msg, self.imu_msg, current_time)


    def obstacleAvoidanceLogic(self):

        # Check all required data for the obstacle logic
        if self.frame is None or self.localization_msg is None or self.imu_msg is None or self.odom_msg is None:
            return

        # Assuming get_swerve() is the method from obstacle avoidance that returns the swerve commands
        self.obstacle_swerve = self.obstacle_specialist.get_swerve(self.frame, self.localization_msg, self.imu_msg, self.odom_msg)


    def autonomousCallback(self):

        arucoSwerve = self.aruco_swerve
        # [0] = linear velocity
        # [1] = angular velocity

        obsAvoidanceSwerve = self.obstacle_swerve
        # [0] = linear velocity
        # [1] = angular velocity
        # [2] = boolean indicating the assured presence of an obstacle

        sw_msg = Float32MultiArray()

        if obsAvoidanceSwerve[2] == False:
            sw_msg.data = [arucoSwerve[0], arucoSwerve[1]]
        else:
            sw_msg.data = [obsAvoidanceSwerve[0], obsAvoidanceSwerve[1]]

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
        autonomous_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
