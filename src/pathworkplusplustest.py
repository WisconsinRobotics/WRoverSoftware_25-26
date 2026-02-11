import depthai as dai
import cv2
import numpy as np
import time
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32MultiArray

from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header

class point_cloud_from_depth(Node):

    def __init__(self):
        super().__init__("point_cloud_publisher")
        self.pc_pub = self.create_publisher(PointCloud2, "/depthpc2", 1)

    def publish_pc(self, points):
        # points is the Nx3 numpy array from generate_point_cloud
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "oak_rgb_camera_optical_frame" # Match your camera frame
        
        # Create the PointCloud2 message
        msg = pc2.create_cloud_xyz32(header, points)
        self.pc_pub.publish(msg)

    def generate_point_cloud(self, depth_frame):
        """
        Converts a depth frame into a 3D Point Cloud (Nx3 array)
        """
        X_PIXEL_OFFSET = 640.0 
        Y_PIXEL_OFFSET = 360.0
        FOCAL_LENGTH = 563.33333
        
        H, W = depth_frame.shape

        # Create a grid of (u, v) pixel coordinates
        # u is horizontal (columns), v is vertical (rows)
        u = np.arange(W)
        v = np.arange(H)
        uu, vv = np.meshgrid(u, v)

        # Z is the depth value itself
        z = depth_frame

        # X = (u - offset) * Z / f
        x = (uu - X_PIXEL_OFFSET) * z / FOCAL_LENGTH
        
        # Y = (offset - v) * Z / f 
        y = (Y_PIXEL_OFFSET - vv) * z / FOCAL_LENGTH
        
        points = np.stack((x, y, z), axis=-1)

        return points
    
with dai.Pipeline() as pipeline:
    monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
    stereo = pipeline.create(dai.node.StereoDepth)

    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.ROBOTICS)
    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
    stereo.setOutputSize(1280, 720)

    config = stereo.initialConfig

    # Median filter to remove the salt n pepper type pixels
    config.postProcessing.median = dai.MedianFilter.KERNEL_7x7
    config.postProcessing.thresholdFilter.maxRange = 8000 # 8.0m

    config.setConfidenceThreshold(30)



    monoLeftOut = monoLeft.requestOutput((1280, 720))
    monoRightOut = monoRight.requestOutput((1280, 720))

    monoLeftOut.link(stereo.left)
    monoRightOut.link(stereo.right)

    rightOut = monoRightOut.createOutputQueue()
    stereoOut = stereo.depth.createOutputQueue()

    rclpy.init()
    obj = point_cloud_from_depth()

    pipeline.start()
    while pipeline.isRunning():
        rclpy.spin_once(obj, timeout_sec=0.0)
        stereoFrame = stereoOut.get()

        assert stereoFrame.validateTransformations()
        
        # Get frame and convert to meters
        depth = stereoFrame.getCvFrame().astype(np.float32) / 1000.0
        
        
        # Call the processing function, now passing the heading
        points = obj.generate_point_cloud(depth)
        obj.publish_pc(points=points)

    pipeline.stop()


cv2.destroyAllWindows()
## --- END: Pipeline and Device Loop ---
