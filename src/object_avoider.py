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
from std_msgs.msg import Float32MultiArray
from gps_heading import HeadingVerifier

class SwervePublisher(Node):
    def __init__(self):
        super().__init__("swerve_publisher")

        self.pub = self.create_publisher(Float32MultiArray, "swerve", 10)

    def send(self, swrv):
        msg = Float32MultiArray()
        msg.data = swrv
        self.pub.publish(msg)


class SectorDepthClassifier():
    def __init__(self):
        self.debug = True
        self.onRover = True
        # This sets up the code to broadcast video to the network
        #if self.debug:
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.setsockopt(zmq.CONFLATE, 1)
        self.socket.bind("tcp://*:9876")  # Binds to port 5555
        print("Video Streamer initialized on port 6000")
        self.previous_best_theta = None
        self.candidate_theta = None
        self.persistence_counter = 0

        # RANSAC cache
        self.cached_n = None
        self.cached_d = None
        self.ransac_counter = 0
        self.RANSAC_INTERVAL = 10 
        self.X_PIXEL_OFFSET = np.float32(640)  #(648.040894)
        self.Y_PIXEL_OFFSET = np.float32(360)
        self.FOCAL_LENGTH = np.float32(563.33333)
        self.GAP_THRESHOLD = np.float32(1.8) # The minimum distance between two obstacles such that the rover can fit.
        self.DEPTH_THRESH = np.float32(3.5)
        self.SAFETY_BUFFER = 0.6
        self.W = 1280
        self.H = 720 

        self.DEG_PER_SECT = 3.0  # degrees per sector
        self.FOV_DEG = 2 * np.degrees(np.arctan(self.W / (2 * self.FOCAL_LENGTH)))
        self.NUM_SECTORS = int(self.FOV_DEG / self.DEG_PER_SECT)

        cols = np.arange(self.W)
        angle_deg = np.degrees(np.arctan((cols - self.X_PIXEL_OFFSET) / self.FOCAL_LENGTH))
        self.col_to_sector = np.clip(
            ((angle_deg + self.FOV_DEG / 2) / self.DEG_PER_SECT).astype(int),
            0, self.NUM_SECTORS - 1
        )
        self.pixel_to_sector = np.tile(self.col_to_sector, (self.H, 1))
        self.pixels_per_sector = np.bincount(
            self.pixel_to_sector.ravel(),
            minlength=self.NUM_SECTORS
        ).astype(float)
        self.pixels_per_sector[self.pixels_per_sector == 0] = 1
            
    
    def get_ground_mask(self, depth):
        self.ransac_counter += 1

        if self.cached_n is None or self.ransac_counter >= self.RANSAC_INTERVAL:
            pts_sub, _, _ = backproject_depth_sub(depth)
            if len(pts_sub) >= 100:
                success, n, d, _ = ransac_plane(
                    pts=pts_sub,
                    iters=300,         
                    dist_thresh=0.08,
                    angle_thresh_deg=45,
                    min_inliers=450,
                )
                if success:
                    self.cached_n = n
                    self.cached_d = d
                    self.ransac_counter = 0

        if self.cached_n is not None:
            return apply_plane_to_full_depth(depth, self.cached_n, self.cached_d)
        return None

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


        # ground_mask = self.get_ground_mask(depth_full)
        # if ground_mask is not None:
        #     # RANSAC found the plane
        #      depth_full[ground_mask] = np.float32(10)
        # else:
        # #     # Fallback to old method
        rows = (self.Y_PIXEL_OFFSET - np.arange(H, dtype=np.float32)) / self.FOCAL_LENGTH
        fallback_mask = depth_full * rows[:, None] < -0.18
        depth_full[fallback_mask] = np.float32(10)
        ground_mask = fallback_mask

        # ---------------------------------------------
        # ------- ground has been removed pog ---------
        # ---------------------------------------------

        magnitudes = np.zeros_like(depth_full)
        obstacle_mask = (
            ~ground_mask &
            ~np.isnan(depth_full) &
            (depth_full > 0.1) &
            (depth_full < self.DEPTH_THRESH)
        )
        magnitudes[obstacle_mask] = (
            (self.DEPTH_THRESH - depth_full[obstacle_mask]) / self.DEPTH_THRESH
        ) ** 2

        raw_hist = np.bincount(
            self.pixel_to_sector.ravel(),
            weights=magnitudes.ravel(),
            minlength=self.NUM_SECTORS
        ).astype(float)

        # ---------------------------------------------
        # ------- magnitudes calculated ez--- ---------
        # ---------------------------------------------

        # ---------------------------------------------
        # ------- SMOOTH THE HISTRAGRAM NOW--- ---------
        # ---------------------------------------------

        # ---------------------------------------------
        # ------- magnitudes calculated ez--- ---------
        # ---------------------------------------------
        
        # ---------------------------------------------
        # ------- visualziation -----------------------
        # ---------------------------------------------

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
            current_heading = -1 * (current_heading) % 360
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
