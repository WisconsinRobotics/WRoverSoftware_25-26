import depthai as dai
import cv2
import numpy as np
import time
import math
from trial_ground_detection import detect_ground_points, draw_line_segments

class SectorDepthClassifier():

    X_PIXEL_OFFSET = np.float32(640)  #(648.040894)
    Y_PIXEL_OFFSET = np.float32(360)
    FOCAL_LENGTH = np.float32(563.33333)
    GAP_THRESHOLD = np.float32(1) # The minimum distance between two obstacles such that the rover can fit.
    DEPTH_THRESH = np.float32(2)

    def cb(self, depth_full):
        start_time = time.time()
        # Decode and crop depth image
      
        mask = (depth_full == 0) | (depth_full == np.nan)
        depth_full[mask] = np.float32(10)
        H,W = depth_full.shape        
        
        rows = (np.arange(depth_full.shape[0], dtype=np.float32) - self.Y_PIXEL_OFFSET) / self.FOCAL_LENGTH
        ground_mask = depth_full * rows[:, None] > 0.5
        depth_full[ground_mask] = np.float32(10)

        # list of all min values of each vertical sector. values are in m
        min_list = np.percentile(depth_full, 8, axis=0)
        
        # newmask = (depth_full == np.nan)
        # depth_full[newmask] = np.float32(10)

        # list of where objects are
        gap_list = (min_list <= self.DEPTH_THRESH).astype(int)

        
        d = np.diff(gap_list)

        starts = np.nonzero(d == -1)[0]
        ends = np.nonzero(d == 1)[0] + 1

        if not gap_list[-1]:
            ends = np.concatenate((ends, [gap_list.size - 1]))
        if not gap_list[0]:
            starts = np.concatenate(([0], starts))

        gaps = list(zip(starts, ends))


        # code is optimized till here

        thetas = []
        distance_monitor_list = []
        for gap in gaps:
            ux1 = gap[0]
            ux2 = gap[1]
            
            theta1 = np.arctan((ux1 - self.X_PIXEL_OFFSET)/self.FOCAL_LENGTH) 
            theta2 = np.arctan((ux2 - self.X_PIXEL_OFFSET)/self.FOCAL_LENGTH)

            d1 = min_list[ux1]/np.cos(theta1)
            d2 = min_list[ux2]/np.cos(theta2)
            
            # Calculating the theta for each gap
            
            theta = theta2 - theta1
            thetas.append(theta)
            gap_distance = np.sqrt(d1**2 + d2**2 - (2*d1*d2*np.cos(theta)))
            distance_monitor_list.append(gap_distance)
        
        formatted_list = [round(float(x), 2) for x in min_list]
        print(formatted_list)
        print("angles====================\n", (np.array(thetas)*180)/3.14) # These are the angles of each gap.
        print("list of gaps =====================\n",gaps)        
        print("list of distance between gaps =================================\n", distance_monitor_list, "\n\n\n")
        
        valid_gaps = []
        for i,d in enumerate(distance_monitor_list):
            if d >= self.GAP_THRESHOLD:
                valid_gaps.append(gaps[i])
                                                                                                                                                                                            
        # Check for the angles
        try:
            gap_to_move_to = valid_gaps[0]
        except IndexError:
            print("no valid gaps u have crashed!!!!!! :)")
            gap_to_move_to = (0,0)

        # for start, end in valid_gaps:
        #     ux1 = start
        #     ux2 = end
            
        #     theta1 = np.arctan((ux1 - self.X_PIXEL_OFFSET)/self.FOCAL_LENGTH) 
        #     theta2 = np.arctan((ux2 - self.X_PIXEL_OFFSET)/self.FOCAL_LENGTH)

        #     d1 = min_list[ux1]/np.cos(theta1)
        #     d2 = min_list[ux2]/np.cos(theta2)

        #     theta = abs(theta2 - theta1) # angle between d1 d2

        #     d3 = np.sqrt(d1**2 + d2**2 - (2*d1*d2*np.cos(theta))) # the distance between objects

        #     phi = np.arccos((d2**2 - d3**2 - d1**2)/2*d3*d1) # angle between d3 and d1

        #     d4 = np.sqrt(self.GAP_THRESHOLD**2 + d1**2 - (self.GAP_THRESHOLD*2*d1*np.cos(phi))) # straight line dropped from the gap to camera

        #     z = np.arccos((self.GAP_THRESHOLD**2 - d4**2 - d1**2)/2*d1*d4) # optimal angle in that gap

        #     if abs(theta) < abs(best_theta):
        #         gap_to_move_to = (start, end)


        # compass_angle: angle from North to heading in the clockwise direction
        # compute_bearing: angle from North to target in the clockwise direction
        # target_angle = (360 - (compass_angle - self.compute_bearing(rover_gps , target_gps))) % 360
        # if target_angle > 180:
        #     target_angle = target_angle - 360
        target_angle = 0
        # replaced the function find_theta()
        for start, end in valid_gaps:
                median = (end - start)//2
                theta = np.arctan((median - self.X_PIXEL_OFFSET)/self.FOCAL_LENGTH)
                
                best_theta = np.arctan(((gap_to_move_to[1] - gap_to_move_to[0])/2 - self.X_PIXEL_OFFSET)/self.FOCAL_LENGTH)

                if abs(target_angle - theta) < abs(target_angle - best_theta):
                    gap_to_move_to = (start, end)
            
        # Appends ground points into the feed
        ground_points = detect_ground_points(frame_with_depth_values=depth_full)

        depth_full = cv2.normalize(depth_full, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        depth_full = cv2.cvtColor(depth_full, cv2.COLOR_GRAY2BGR)

        depth_full[ground_mask] = (255, 0, 0)

        for gap in valid_gaps:
            start_point, end_point = (gap[0], 0), (gap[1], 719)
            color = (0, 255, 0)
            depth_full = cv2.rectangle(depth_full, start_point, end_point, color, -1)

            # Publish overlay

        start_point, end_point = (gap_to_move_to[0], 0), (gap_to_move_to[1], 719)
        color = (0, 255, 255)
        depth_full = cv2.rectangle(depth_full, start_point, end_point, color, -1)

        # Apply the line for ground
        depth_full = draw_line_segments(image=depth_full, segments=ground_points)

        cv2.imshow("obstacle avoidance", depth_full)
        cv2.waitKey(1)
        
        end_time = time.time() - start_time
        print(end_time)

    

    def compute_bearing(p1, p2):
        """
        Computes the angle between two gps coordinates in degrees

        Args:
            p1 - first gps coordinate
            p2 - second gps coordinate
        Returns:
            angle with respect to north that points into the direction
        """
        lat1, lon1 = p1
        lat2, lon2 = p2
        
        # Convert degrees to radians
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)

        # Calculate differences in coordinates
        dlon = lon2_rad - lon1_rad

        # Calculate bearing using atan2
        x = math.sin(dlon) * math.cos(lat2_rad)
        y = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon)

        bearing_rad = math.atan2(x, y)

        # Convert bearing from radians to degrees (0° to 360°)
        bearing_deg = math.degrees(bearing_rad)
        bearing_deg = (bearing_deg + 360) % 360  # Normalize to 0-360

        return bearing_deg

with dai.Pipeline() as pipeline:
    monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
    stereo = pipeline.create(dai.node.StereoDepth)

    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.ROBOTICS)
    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
    stereo.setOutputSize(1280, 720)
    
    config = stereo.initialConfig

    # Median filter to remove the salt n pepper type pixels


    # Temporal Filter for flicker reduction
    # config.postProcessing.temporalFilter.enable = True
    # config.postProcessing.temporalFilter.alpha = 0.3
    # config.postProcessing.temporalFilter.delta = 5
    # config.postProcessing.temporalFilter.persistencyMode = dai.StereoDepthConfig.PostProcessing.TemporalFilter.PersistencyMode.VALID_2_IN_LAST_4

    # Threshold Filter to remove invalid '0' pixels and set an operational range
    # config.postProcessing.thresholdFilter.minRange = 300  # 30cm
    config.postProcessing.thresholdFilter.maxRange = 8000 # 8.0m

    config.setConfidenceThreshold(50)



    monoLeftOut = monoLeft.requestOutput((1280, 720))
    monoRightOut = monoRight.requestOutput((1280, 720))

    monoLeftOut.link(stereo.left)
    monoRightOut.link(stereo.right)

    rightOut = monoRightOut.createOutputQueue()
    stereoOut = stereo.depth.createOutputQueue()
    
    obj = SectorDepthClassifier()

    pipeline.start()
    while pipeline.isRunning():
        stereoFrame = stereoOut.get()

        assert stereoFrame.validateTransformations()
        # depth = processDepthFrame(stereoFrame.getCvFrame())
        depth = stereoFrame.getCvFrame().astype(np.float32) / 1000.0
        obj.cb(depth)
        
    pipeline.stop()
