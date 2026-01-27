import depthai as dai
import cv2
import numpy as np
import time
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32MultiArray




class SectorDepthClassifier():

    X_PIXEL_OFFSET = np.float32(605)  #(648.040894)
    Y_PIXEL_OFFSET = np.float32(360)
    FOCAL_LENGTH = np.float32(563.33333)
    GAP_THRESHOLD = np.float32(0.5) # The minimum distance between two obstacles such that the rover can fit.
    DEPTH_THRESH = np.float32(2)

    def topDownMap(self, depth_full):

        mask = (depth_full == 0) | np.isnan(depth_full)
        depth_full[mask] = 8.0

        h, w = depth_full.shape

        # --- 2. VECTORIZED 3D CALCULATION ---
        # Create grid of indices: i = x (cols), j = y (rows)
        i_grid, j_grid = np.meshgrid(np.arange(w), np.arange(h))

        Z = depth_full
        
        X_real = Z * (i_grid - self.X_PIXEL_OFFSET) / self.FOCAL_LENGTH
        
        Y_real = Z * (j_grid - self.Y_PIXEL_OFFSET) / self.FOCAL_LENGTH

        # GROUND DETECTION FIX
        # Since Y is positive downwards:
        # Floor is everything BELOW the camera (Y > threshold)
        # Ceiling is everything ABOVE the camera (Y < -threshold)
        # Adjust '0.4' to your actual camera height in meters.
        floor_mask = Y_real > 0.4
        ceiling_mask = Y_real < -1.5
        

        MAP_SCALE = 100  # 100 px = 1 meter
        MAP_W, MAP_H = 1001, 801
        MAP_CX = 500
        MAP_CY = 800

        map_x = (X_real * MAP_SCALE + MAP_CX).astype(np.int32)
        map_y = (MAP_CY - (Z * MAP_SCALE)).astype(np.int32)

        # Filter Valid Points:
        #Inside map bounds
        # NOT the floor and NOT the ceiling
        valid_indices = (
            (map_x >= 0) & (map_x < MAP_W) & 
            (map_y >= 0) & (map_y < MAP_H) & 
            (~floor_mask) & (~ceiling_mask) &
            (Z < 9.5)
        )


        map_image = np.zeros((MAP_H, MAP_W, 3), dtype=np.uint8)
        # Draw obstacles in Green
        map_image[map_y[valid_indices], map_x[valid_indices]] = (0, 255, 0)
        # Draw Camera in Red
        cv2.rectangle(map_image, (MAP_CX-5, MAP_CY-5), (MAP_CX+5, MAP_CY), (0, 0, 255), -1)


        # Normalize depth to 0-255 for visibility (Range 0m to 5m)
        depth_vis = np.clip(depth_full, 0, 5.0)
        depth_vis = (depth_vis / 5.0 * 255).astype(np.uint8)
        
        # Make it color (BGR) so we can draw blue
        camera_view = cv2.cvtColor(depth_vis, cv2.COLOR_GRAY2BGR)
        
        # Highlight the detected ground in BLUE
        # If this highlights the walls/obstacles, increase  the 0.4 threshold above
        camera_view[floor_mask] = (255, 0, 0) 
        
        #DISPLAY WINDOWS
        cv2.imshow("Debug: Camera View", camera_view)
        cv2.imshow("Top Down Map", map_image)
        cv2.waitKey(1)
        return
                


    
with dai.Pipeline() as pipeline:
    monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
    stereo = pipeline.create(dai.node.StereoDepth)

    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.ROBOTICS)
    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
    stereo.setOutputSize(1280, 720)

    config = stereo.initialConfig

    # Median filter to remove the salt n pepper type pixels
    # config.postProcessing.median = dai.MedianFilter.KERNEL_5x5
    config.postProcessing.thresholdFilter.maxRange = 8000 # 8.0m

    config.setConfidenceThreshold(50)


    monoLeftOut = monoLeft.requestOutput((1280, 720))
    monoRightOut = monoRight.requestOutput((1280, 720))

    monoLeftOut.link(stereo.left)
    monoRightOut.link(stereo.right)

    rightOut = monoRightOut.createOutputQueue()
    stereoOut = stereo.depth.createOutputQueue()

    obj = SectorDepthClassifier()

    #swerve_queue = np.array() # TODO FINSIH THIS TODODODODODO
    pipeline.start()
    while pipeline.isRunning():
        stereoFrame = stereoOut.get()
        assert stereoFrame.validateTransformations()
        # Get frame and convert to eters
        depth = stereoFrame.getCvFrame().astype(np.float32) / 1000.0
        
        
        # Call the processing function, now passing the heading
        obj.topDownMap(depth)
    

    pipeline.stop()


cv2.destroyAllWindows()