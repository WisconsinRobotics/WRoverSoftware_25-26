import depthai as dai
import cv2
import numpy as np
import time
import math
import zmq
import base64

from ground_detect import backproject_depth_sub
from ground_detect import ransac_plane
from ground_detect import apply_plane_to_full_depth
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64
from gps_heading import HeadingVerifier

class SectorDepthClassifier():
    def __init__(self):
        self.W = 1280
        self.H = 720
        self.FOCAL_LENGTH = 563.33333
        self.X_PIXEL_OFFSET = 640
        self.ALPHA_DEG = 3.0   # degrees per sector
        self.FOV_DEG = 2 * np.degrees(np.arctan(self.W / (2 * self.FOCAL_LENGTH)))
        self.NUM_SECTORS = int(self.FOV_DEG / self.ALPHA_DEG)
        print("NUM SECTORS = ", self.NUM_SECTORS)

        cols = np.arange(self.W)
        angle_deg = np.degrees(np.arctan((cols - self.X_PIXEL_OFFSET) / self.FOCAL_LENGTH))
        self.col_to_sector = np.clip(
            ((angle_deg + self.FOV_DEG / 2) / self.ALPHA_DEG).astype(int),
            0, self.NUM_SECTORS - 1
        )
        self.pixel_to_sector = np.tile(self.col_to_sector, (self.H, 1))
        self.pixels_per_sector = np.bincount(
            self.pixel_to_sector.ravel(),
            minlength=self.NUM_SECTORS
        ).astype(float)
        self.pixels_per_sector[self.pixels_per_sector == 0] = 1
        
        
    def cb(self, depth):
        depth_vis = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        depth_vis = cv2.cvtColor(depth_vis, cv2.COLOR_GRAY2BGR)

        # alternate colors per sector
        colors = [[0, 0, 255], [0, 255, 0]]  # cyan, green
        for i in range(self.W):
            sector = self.col_to_sector[i]
            depth_vis[:, i] = colors[sector % 2]

        cv2.imshow("sector viz", depth_vis)  # was showing raw depth, not depth_vis
        cv2.waitKey(0)


with dai.Pipeline() as pipeline:
    monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
    stereo = pipeline.create(dai.node.StereoDepth)

    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.ROBOTICS)
    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_B)
    stereo.setOutputSize(1280, 720)

    config = stereo.initialConfig

    # Median filter to remove the salt n pepper type pixels
    config.postProcessing.median = dai.MedianFilter.KERNEL_7x7
    config.postProcessing.thresholdFilter.maxRange = 8000 # 8.0m

    config.setConfidenceThreshold(50)
    config.setSubpixel(True)
    config.setExtendedDisparity(True)

    monoLeftOut = monoLeft.requestOutput((1280, 720))
    monoRightOut = monoRight.requestOutput((1280, 720))

    monoLeftOut.link(stereo.left)
    monoRightOut.link(stereo.right)

    rightOut = monoRightOut.createOutputQueue()
    stereoOut = stereo.depth.createOutputQueue()
    
    imu = pipeline.create(dai.node.IMU)
    imu.enableIMUSensor(dai.IMUSensor.ARVR_STABILIZED_ROTATION_VECTOR, 100) # 100 Hz
    imu.setBatchReportThreshold(1)
    imu.setMaxBatchReports(10)
    imuQueue = imu.out.createOutputQueue(maxSize=10, blocking=False)

    obj = SectorDepthClassifier()

    rclpy.init()
    # imu_node = IMUNode()
    #swerve_queue = np.array() # TODO FINSIH THIS TODODODODODO
    verifier = HeadingVerifier(min_move_dist=1.0, alpha=0.2)
    pipeline.start()
    current_heading = 0.0
    while pipeline.isRunning():
        # rclpy.spin_once(imu_node, timeout_sec=0.0)
        # msg = imu_node.latest_imu

        # q_x = msg.x
        # q_y = msg.y
        # q_z = msg.z
        # q_w = msg.w

        # current_heading = quaternion_to_yaw(q_x, q_y, q_z, q_w)
        # print("uncorrected heeading relative to north = ", current_heading)

        # final_heading = verifier.get_corrected_heading(
        #      current_imu=imu_node.latest_imu, 
        #      current_gps=gps_node.latest_gps
        # )

        # current_heading = final_heading

        stereoFrame = stereoOut.get()
        # print("corrected heeading relative to north = ", current_heading)
        assert stereoFrame.validateTransformations()
        
        # Get frame and convert to meters
        depth = stereoFrame.getCvFrame().astype(np.float32) / 1000.0
        
        
        # Call the processing function, now passing the heading
        obj.cb(depth)
        # if cv2.waitKey(1) == ord('q'):
        #     break
        

    pipeline.stop()


cv2.destroyAllWindows()
## --- END: Pipeline and Device Loop ---
