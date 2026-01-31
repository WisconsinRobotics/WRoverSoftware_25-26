import depthai as dai
import cv2
import numpy as np
import time
import math
import zmq
import base64

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64
from gps_heading import HeadingVerifier

class SwervePublisher(Node):
    def __init__(self):
        super().__init__("swerve_publisher")

        self.pub = self.create_publisher(Float32MultiArray, "swerve", 10)

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
            10
        )

    def gps_callback(self, msg):
        self.latest_gps = (msg.latitude, msg.longitude)

class IMUNode(Node):
    def __init__(self):
        super().__init__("imu_listener")

        self.latest_imu = 0.0

        self.create_subscription(Float64, "compass_data_topic", self.imu_cb, 10)
        
    def imu_cb(self, msg):
        self.latest_imu = msg.orientation


class SectorDepthClassifier():
    def __init__(self):
        self.debug = True

        # self.commit_frames = 6    
        # self.lock_counter = 0      
        # self.last_command = [0.0, 0.0, 0.0, 0.0]
        # This sets up the code to broadcast video to the network
        if self.debug:
            self.context = zmq.Context()
            self.socket = self.context.socket(zmq.PUB)
            self.socket.setsockopt(zmq.CONFLATE, 1)
            self.socket.bind("tcp://*:8050")  # Binds to port 5555
            print("Video Streamer initialized on port 6000")

    X_PIXEL_OFFSET = np.float32(640)  #(648.040894)
    Y_PIXEL_OFFSET = np.float32(360)
    FOCAL_LENGTH = np.float32(563.33333)
    GAP_THRESHOLD = np.float32(1.65) # The minimum distance between two obstacles such that the rover can fit.
    DEPTH_THRESH = np.float32(2.65)
    SAFETY_BUFFER = 1.0   
        
    ## CHANGED: Added 'compass_angle' as an argument
    def cb(self, depth_full, compass_angle, rover_gps):
        start_time = time.time()
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
        min_list = np.percentile(depth_full, 6, axis=0)

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
        # formatted_list = [round(float(x), 2) for x in min_list]
        # print(formatted_list)
        # print("angles====================\n", (np.array(thetas)*180)/3.14) # These are the angles of each gap.
        # print("list of gaps =====================\n",gaps)        
        # print("list of distance between gaps =================================\n", distance_monitor_list, "\n\n\n")

        ## --- START: IMU Target Angle Implementation ---
        ## CHANGED: This block now uses the live 'compass_angle'

        # compass_angle: angle from North to heading in the clockwise direction (0-360)
        # This is now passed into the function.

        # ** You must update these with your live GPS data **
        rover_gps = (43.073107912662266, -89.41245993537561)
        target_gps = (43.0724831900561, -89.41245993537561)

        # compute_bearing: angle from North to target in the clockwise direction
        bearing_to_target = self.compute_bearing(rover_gps , target_gps)
        bearing_to_target = 0 # always moves north if 0 east if 90 and so on
        # Calculate the relative angle the rover needs to turn to
        # diff: how many degrees we need to turn from current heading to hit bearing
        target_angle_deg = (360 - (compass_angle - bearing_to_target)) % 360
        if target_angle_deg > 180:
            target_angle_deg = target_angle_deg - 360  
        target_angle_rad = math.radians(target_angle_deg) 
        print("target angle = ", target_angle_deg)
        # 5. Process Gaps and find Best Steering Angle
        valid_gaps = []
        best_theta = 999.0
        gap_to_move_to = (0, 0)

        for gap in gaps:
            ux1, ux2 = gap[0], gap[1]
            
            # Pessimistic Width Logic
            z = min(min_list[ux1], min_list[min(ux2, W-1)])
            theta_raw1 = np.arctan((ux1 - self.X_PIXEL_OFFSET) / self.FOCAL_LENGTH) 
            theta_raw2 = np.arctan((ux2 - self.X_PIXEL_OFFSET) / self.FOCAL_LENGTH)
            
            # Law of Cosines for pessimistic width
            d1 = min_list[ux1]/np.cos(theta_raw1)
            d2 = min_list[ux2]/np.cos(theta_raw2)

            d = min(d2,d1)
            width = np.sqrt(2 * (d**2) * (1 - np.cos(abs(theta_raw2 - theta_raw1))))

            if width >= self.GAP_THRESHOLD:
                # Apply Safety Buffers (Physical meters -> Pixel space)
                xStart = (z * (ux1 - self.X_PIXEL_OFFSET) / self.FOCAL_LENGTH)
                xEnd = (z * (ux2 - self.X_PIXEL_OFFSET) / self.FOCAL_LENGTH)
                
                safe_xStart = xStart + self.SAFETY_BUFFER
                safe_xEnd = xEnd - self.SAFETY_BUFFER
                
                safe_px_start = (safe_xStart * self.FOCAL_LENGTH / z) + self.X_PIXEL_OFFSET
                safe_px_end = (safe_xEnd * self.FOCAL_LENGTH / z) + self.X_PIXEL_OFFSET

                if safe_px_start < safe_px_end:
                    valid_gaps.append((round(safe_px_start), round(safe_px_end)))
                    
                    # Find best point in this gap
                    theta_safe1 = np.arctan((safe_px_start - self.X_PIXEL_OFFSET) / self.FOCAL_LENGTH)
                    theta_safe2 = np.arctan((safe_px_end - self.X_PIXEL_OFFSET) / self.FOCAL_LENGTH)
                    
                    # Target Clamping (Replaces your pixel loop)
                    clamped_theta = max(theta_safe1, min(target_angle_rad, theta_safe2))

                    if abs(target_angle_rad - clamped_theta) < abs(target_angle_rad - best_theta):
                        best_theta = clamped_theta
                        gap_to_move_to = (round(safe_px_start), round(safe_px_end))

        # 6. Visualization Block
        if self.debug:
            depth_vis = cv2.normalize(depth_full, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
            depth_vis = cv2.cvtColor(depth_vis, cv2.COLOR_GRAY2BGR)

            # Paint Ground Blue
            depth_vis[ground_mask] = (255, 0, 0)

            # Paint Valid Gaps Green
            for gap in valid_gaps:
                cv2.rectangle(depth_vis, (int(gap[0]), 0), (int(gap[1]), H-1), (0, 255, 0), -1)

            # Paint Chosen Gap Yellow (Drawn over green)
            if best_theta != 999.0:
                cv2.rectangle(depth_vis, (int(gap_to_move_to[0]), 0), (int(gap_to_move_to[1]), H-1), (0, 255, 255), -1)
                
                # Paint Chosen Pixel Column Red
                chosen_pixel = int((math.tan(best_theta) * self.FOCAL_LENGTH) + self.X_PIXEL_OFFSET)
                cv2.line(depth_vis, (chosen_pixel, 0), (chosen_pixel, H-1), (0, 0, 255), 3)
            
            # cv2.imshow("Aasd", depth_vis)
            # cv2.waitKey(1)

            try:
                # Compress to jpg to save bandwidth
                ret, buffer = cv2.imencode('.jpg', depth_vis, [int(cv2.IMWRITE_JPEG_QUALITY), 30])
                if ret:
                    # 2. Convert to Base64 (This is what the receiver expects)
                    jpg_as_text = base64.b64encode(buffer.tobytes())

                    # 3. Send as a single message (Receiver uses recv(), not recv_multipart())
                    self.socket.send(jpg_as_text)
                    # print("--------------------------sent frame -----------------------------")
            except Exception as e:
                print(f"")

            # cv2.imshow("obstacle avoidance", depth_full)
            # cv2.waitKey(1)
        
        if best_theta == 999.0:
            print("YOUR HAVE CRASHED NO VALID GAPSS S ---- :))))")
            # Return a "Stop" command [Speed, Angle, LeftMotor, RightMotor]
            if(target_angle_deg > 0): 
                return [ 0.0 , 0.0, -0.5, 0.5] 
            else:
                return [0.0, 0.0, 0.5, -0.5]

        end_time = time.time() - start_time
        # print("time :",end_time)
        error = math.degrees(best_theta)
        print("error :", error)
        kP = 0.02  # Tune this: higher = faster turns, lower = smoother
        min_speed = 0.2
        max_speed = 1.0
        
        speed = error * kP
        
        if abs(error) < 6.5: # range to move forward
            speed = ((min_list[640]+min_list[639]+min_list[641])/3) * 0.09 # moves slower if there is more stuff in front of it 
            print("speed = ", speed)
            return [float(speed), 0.0,-1.0,-1.0] # Drive forward
        else:
        # If speed is too low the robot won't move, so add a floor
            if speed < 0:
                speed = abs(speed)
                if abs(speed) < min_speed: 
                    speed = math.copysign(min_speed, speed)
                return [0.0,0.0,speed, -1.0] 
            else:
                if abs(speed) < min_speed: 
                    speed = math.copysign(min_speed, speed)
                return [0.0,0.0, -1.0, speed] 

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
    gps_node = GPSNode()
    swerve_node = SwervePublisher()
    # imu_node = IMUNode()
    #swerve_queue = np.array() # TODO FINSIH THIS TODODODODODO
    verifier = HeadingVerifier(min_move_dist=1.0, alpha=0.2)
    pipeline.start()
    current_heading = 0.0
    while pipeline.isRunning():
        rclpy.spin_once(gps_node, timeout_sec=0.0)
        rclpy.spin_once(swerve_node, timeout_sec=0.0)
        # rclpy.spin_once(imu_node, timeout_sec=0.0)


        imuData = imuQueue.tryGet()
        if imuData:
            imuPacket = imuData.packets[-1]
            rv = imuPacket.rotationVector
            current_heading = quaternion_to_yaw(rv.i, rv.j, rv.k, rv.real)
            current_heading = -1 * (current_heading - 183.5) % 360
            print("current heeading relative to north = ", current_heading)
        ## --- Depth Data Processing ---
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
        swerve_cmd = obj.cb(depth, current_heading, gps_node.latest_gps)
    

        swerve_node.send(swerve_cmd)

        # if cv2.waitKey(1) == ord('q'):
        #     break
        

    pipeline.stop()


cv2.destroyAllWindows()
## --- END: Pipeline and Device Loop ---
