import depthai as dai
import cv2
import numpy as np
import time
import math
import zmq

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64
from gps_heading import HeadingVerifier

class SwervePublisher(Node):
    def __init__(self):
        super().__init__("swerve_publisher")

        self.pub = self.create_publisher(Float32MultiArray, "swerve", 1)

    def send(self, swrv):
        msg = Float32MultiArray()
        msg.data = swrv
        self.pub.publish(msg)

class GPSNode(Node):
    def __init__(self):
        super().__init__("gps_listener")

        self.latest_gps = (0,0)  # (lat, lon)

        self.create_subscription(
            NavSatFix,
            "fix",          
            self.gps_callback,
            1
        )

    def gps_callback(self, msg):
        self.latest_gps = (msg.latitude, msg.longitude)

class IMUNode(Node):
    def __init__(self):
        super().__init__("imu_listener")

        self.latest_imu = 0.0

        self.create_subscription(Float64, "compass_data_topic", self.imu_cb, 1)
        
    def imu_cb(self, msg):
        self.latest_imu = (msg.data - 270) % 360


class SectorDepthClassifier():
    def __init__(self):
        # This sets up the code to broadcast video to the network
        context = zmq.Context()
        self.socket = context.socket(zmq.PUB)
        self.socket.bind("tcp://*:8000")  # Binds to port 5555
        print("Video Streamer initialized on port 8000")

    X_PIXEL_OFFSET = np.float32(640)  #(648.040894)
    Y_PIXEL_OFFSET = np.float32(360)
    FOCAL_LENGTH = np.float32(563.33333)
    GAP_THRESHOLD = np.float32(1.9) # The minimum distance between two obstacles such that the rover can fit.
    DEPTH_THRESH = np.float32(2.85)
    SAFETY_BUFFER = 0.4   
        
    ## CHANGED: Added 'compass_angle' as an argument
    def cb(self, depth_full, compass_angle, rover_gps, debug_img=False):
        start_time = time.time()
        # Decode and crop depth image
      
        mask = (depth_full == 0) | np.isnan(depth_full)
        depth_full[mask] = 10.0
        H,W = depth_full.shape        
        
        # start_col = 35
        # end_col = W - 35

        # # Perform the crop using NumPy slicing:
        # # [All Rows, Start Column : End Column]
        # depth_full = depth_full[:, start_col:end_col]
        

        rows = (self.Y_PIXEL_OFFSET - np.arange(depth_full.shape[0], dtype=np.float32)) / self.FOCAL_LENGTH
        # maybe constant optimize? ^^^
        ground_mask = depth_full * rows[:, None] < -0.3
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
            ux1_raw = gap[0]
            ux2_raw = gap[1]
            
            # --- CHANGE 1: EDGE HANDLING (VIRTUAL EXTENSION) ---
            # If the gap touches the edge of the image, we pretend it is 500 pixels wider.
            # This allows the 'median' of the gap to shift toward the target even if 
            # the target is currently off-screen.
            ux1 = -500 if ux1_raw <= 5 else ux1_raw
            ux2 = (W + 500) if ux2_raw >= (W - 5) else ux2_raw

            theta1 = np.arctan((ux1 - self.X_PIXEL_OFFSET)/self.FOCAL_LENGTH) 
            theta2 = np.arctan((ux2 - self.X_PIXEL_OFFSET)/self.FOCAL_LENGTH)

            d1 = min_list[ux1_raw]/np.cos(theta1)
            d2 = min_list[ux2_raw]/np.cos(theta2)
            
            d = min(d2,d1) # changed to finding parrelel disctance to optical plane isntead of straightline distnace between edge of objects
            # Calculating t\\\\\eta for each gap
            
            theta = theta2 - theta1
            thetas.append(theta)
            gap_distance = np.sqrt(d**2 + d**2 - (2*d*d*np.cos(theta)))
            distance_monitor_list.append(gap_distance)
        
        # formatted_list = [round(float(x), 2) for x in min_list]
        # print(formatted_list)
        # print("angles====================\n", (np.array(thetas)*180)/3.14) # These are the angles of each gap.
        # print("list of gaps =====================\n",gaps)        
        # print("list of distance between gaps =================================\n", distance_monitor_list, "\n\n\n")
        
        valid_gaps = []
        checked = []
# --- CHANGE 2: IMPROVED SAFETY BUFFER & DISCARD LOGIC ---
        for i, dist in enumerate(distance_monitor_list):
            # Only consider gaps wider than our minimum threshold
            if dist >= self.GAP_THRESHOLD:
                oldStart = gaps[i][0]
                oldEnd = gaps[i][1]
                z = min(min_list[oldStart], min_list[oldEnd])
                
                # Convert pixel edges to physical meters (X-axis)
                xStart = (z * (oldStart - self.X_PIXEL_OFFSET) / self.FOCAL_LENGTH)
                xEnd = (z * (oldEnd - self.X_PIXEL_OFFSET) / self.FOCAL_LENGTH)
                
                # Apply the safety buffer (moving the target points inward)
                newStart_phys = xStart + self.SAFETY_BUFFER
                newEnd_phys = xEnd - self.SAFETY_BUFFER
                
                # Convert back to pixel coordinates
                newStart = ((newStart_phys) * self.FOCAL_LENGTH / z) + self.X_PIXEL_OFFSET
                newEnd = ((newEnd_phys) * self.FOCAL_LENGTH / z) + self.X_PIXEL_OFFSET
                
                # If newStart < newEnd, the gap is still wide enough for the rover
                if newStart < newEnd:
                    valid_gaps.append((round(newStart), round(newEnd)))
                    checked.append(True)
                else:
                    # Gap was too small after adding safety buffers. Discard it!
                    continue 

        ## --- START: IMU Target Angle Implementation ---
        ## CHANGED: This block now uses the live 'compass_angle'
        
        # compass_angle: angle from North to heading in the clockwise direction (0-360)
        # This is now passed into the function.
        
        # ** You must update these with your live GPS data **
        rover_gps = (43.073107912662266, -89.41245993537561)
        target_gps = (43.0724831900561, -89.41245993537561)

        # compute_bearing: angle from North to target in the clockwise direction
        bearing_to_target = self.compute_bearing(rover_gps , target_gps)
        
        # Calculate the relative angle the rover needs to turn to
        target_angle_deg = (360 - (compass_angle - bearing_to_target)) % 360
        if target_angle_deg > 180:
            target_angle_deg = target_angle_deg - 360  
        # target_angle_deg is currently -ve for right of camera and +ve for left of camera
        # print("target angle= ", -1 * target_angle_deg)
        # Convert target angle from degrees to radians for comparison with arctan result
        target_angle = -1 * math.radians(target_angle_deg) # flip signs
                                                                                                                                                                                                          
        # Check for the angles
        try:
            gap_to_move_to = valid_gaps[0]
        except IndexError:
            print("no valid gaps u have crashed!!!!!! :)")
            return [-0.3, 0.0, -1.0, -1.0]
            gap_to_move_to = (0,0)
        # Optional: Uncomment to debug your angles
        # print(f"Heading: {compass_angle:.1f} | Bearing: {bearing_to_target:.1f} | Target Angle: {target_angle_deg:.1f}")
        
        best_theta = 999.0
        # Now, loop through all gaps and find the one closest to our target_angle
        for (start, end) in valid_gaps:
            median = start + (end - start) // 2 
            theta = np.arctan((median - self.X_PIXEL_OFFSET)/self.FOCAL_LENGTH)

            if abs(target_angle - theta) < abs(target_angle - best_theta):
                gap_to_move_to = (start, end)
                best_theta = theta 

        # print("chosen pixel: ", chosen)            
        # print("chosen angle to drive to: ", math.degrees(best_theta))
            
        if debug_img:
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


            try:
                # Compress to jpg to save bandwidth
                ret, buffer = cv2.imencode('.jpg', depth_full, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
                if ret:
                    self.socket.send(buffer)
            except Exception as e:
                print(f"Stream Error: {e}")

            # cv2.imshow("obstacle avoidance", depth_full)
            # cv2.waitKey(1)
        
        end_time = time.time() - start_time
        print("time :",end_time)
        error = math.degrees(best_theta)
        print("error :", error)
        kP = 0.02  # Tune this: higher = faster turns, lower = smoother
        min_speed = 0.2
        max_speed = 1.0
        
        speed = error * kP
        
        if abs(error) < 7.0: # range to move forward
            return [0.7, 0.0, -1.0, -1.0] # Drive forward
        else:
        # If speed is too low the robot won't move, so add a floor
            if speed < 0:
                speed = abs(speed)
                if abs(speed) < min_speed: 
                    speed = math.copysign(min_speed, speed)
                return [0.0, 0.0, speed, -1.0] 
            else:
                if abs(speed) < min_speed: 
                    speed = math.copysign(min_speed, speed)
                return [0.0, 0.0, -1.0, speed] 

    @staticmethod
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


def quaternion_to_yaw(rv_x, rv_y, rv_z, rv_w):
    """
    Converts a quaternion (Rotation Vector: x, y, z, w) to the yaw angle (in radians).
    
    Args:
        rv_x, rv_y, rv_z: Quaternion vector components (i, j, k from DepthAI).
        rv_w: Quaternion scalar component (real from DepthAI).
        
    Returns:
        float: Yaw angle in degrees (0 to 360).
    """
    
    # Using the standard math formula for yaw (rotation around the Z-axis)
    siny_cosp = 2 * (rv_w * rv_z + rv_x * rv_y)
    cosy_cosp = 1 - 2 * (rv_y * rv_y + rv_z * rv_z)
    
    yaw_rad = math.atan2(siny_cosp, cosy_cosp)
    
    # Convert from radians to degrees
    yaw_deg = math.degrees(yaw_rad)
    
    # Normalize the angle from [-180, 180] to [0, 360]
    heading_360 = (yaw_deg + 360) % 360
    
    return heading_360




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

    config.setConfidenceThreshold(30)

    monoLeftOut = monoLeft.requestOutput((1280, 720))
    monoRightOut = monoRight.requestOutput((1280, 720))

    monoLeftOut.link(stereo.left)
    monoRightOut.link(stereo.right)

    rightOut = monoRightOut.createOutputQueue()
    stereoOut = stereo.depth.createOutputQueue()
    
    # imu = pipeline.create(dai.node.IMU)
    # imu.enableIMUSensor(dai.IMUSensor.GAME_ROTATION_VECTOR, 100) # 100 Hz
    # imu.setBatchReportThreshold(1)
    # imu.setMaxBatchReports(10)
    # imuQueue = imu.out.createOutputQueue(maxSize=10, blocking=False)

    obj = SectorDepthClassifier()

    rclpy.init()
    gps_node = GPSNode()
    swerve_node = SwervePublisher()
    imu_node = IMUNode()
    #swerve_queue = np.array() # TODO FINSIH THIS TODODODODODO
    verifier = HeadingVerifier(min_move_dist=1.0, alpha=0.2)
    pipeline.start()
    current_heading = 0.0
    while pipeline.isRunning():
        rclpy.spin_once(gps_node, timeout_sec=0.0)
        rclpy.spin_once(swerve_node, timeout_sec=0.0)
        rclpy.spin_once(imu_node, timeout_sec=0.0)

        final_heading = verifier.get_corrected_heading(
            current_imu=imu_node.latest_imu, 
            current_gps=gps_node.latest_gps
        )

        # imuData = imuQueue.tryGet()
        # if imuData:
        #     imuPacket = imuData.packets[-1]
        #     rv = imuPacket.rotationVector
        #     current_heading = quaternion_to_yaw(rv.i, rv.j, rv.k, rv.real)
        #     current_heading = (current_heading + 270) % 360
        #     print("current heeading relative to north = ", current_heading)
        ## --- Depth Data Processing ---


        current_heading = imu_node.latest_imu
        print("current heeading relative to north = ", current_heading)

        stereoFrame = stereoOut.get()

        assert stereoFrame.validateTransformations()
        
        # Get frame and convert to meters
        depth = stereoFrame.getCvFrame().astype(np.float32) / 1000.0
        
        
        # Call the processing function, now passing the heading
        swerve_cmd = obj.cb(depth, current_heading, gps_node.latest_gps)
    

        swerve_node.send(swerve_cmd)

        # if cv2.waitKey(1) == ord('q'):
        #     break
        

    pipeline.stop()


cv2.destroyAllWindows()
## --- END: Pipeline and Device Loop ---
