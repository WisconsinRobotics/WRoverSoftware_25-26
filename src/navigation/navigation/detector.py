import cv2 as cv
import depthai as dai
import numpy as np
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from ublox_ubx_msgs.msg import UBXNavPVT


class CameraHandler(Node):
    """
        This class will handle the usage of depthAI camera hardware for the purposes of both object and aruco computer vision
        This will feed into another ROS node
    """
    def __init__(self):
        super().__init__('camera_handler_node')
        self.CAMERA_WIDTH = 1280
        self.CAMERA_HEIGHT = 720

        self.K = np.array([[568.15, 0.0, 643.2372],
                           [0.0, 568.15, 367.1311],
                           [0.0, 0.0, 1.0]], dtype=np.float32)
        
        self.DISTORTION_COEFFS = np.array([2.9425, 0.7698, -3.687e-05, 0.000175,
                                           0.00862, 3.298, 1.626, 0.0925,
                                           0.0, 0.0, 0.0, 0.0, -0.00208, 0.00402], dtype=np.float32)
        
        self.waypoint_sub = self.create_subscription(Float64MultiArray, '/waypoint', self.waypoint_cb, 10)
        self.current_pos_sub = self.create_subscription(UBXNavPVT, '/rover1/ubx_nav_pvt', self.current_pos_cb, 10)
        self.heading_sub = self.create_subscription(Float32, '/heading', self.heading_cb, 10)
        
        self.drive_pub = self.create_publisher(Float32MultiArray, "/swerve", 10)
        self.signal_pub = self.create_publisher(Bool, "/reached_signal", 10)

        # Matched properties from Navigator Node
        self.MAX_FORWARD_SPEED = 0.8 
        self.KP_TURN = 0.015
        self.TELEMETRY_TIMEOUT = 5.0 # Seconds

        self.target_lat = None
        self.target_lon = None
        self.current_lat = None
        self.current_lon = None
        self.heading = None

        self.last_gps_time = self.get_clock().now()
        self.last_heading_time = self.get_clock().now()

        # Aruco defs
        self.arucoDict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
        self.arucoParams = cv.aruco.DetectorParameters()
        self.arucoDetector = cv.aruco.ArucoDetector(self.arucoDict, self.arucoParams)
        
        # Depth AI pipeline
        self.pipeline = dai.Pipeline()
        cam = self.pipeline.create(dai.node.Camera).build()
        self.video_queue = cam.requestOutput((self.CAMERA_WIDTH, self.CAMERA_HEIGHT)).createOutputQueue()
        self.pipeline.start()

        # Timer to run the control loop constantly (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Detector Node initialized (Proportional Mode)...")

    def get_latest_detection(self):
        """
            Polls the depth camera and returns an (offset_x, distance, id) of the detected best tag, or None
        """
        if not self.video_queue.has():
            return None
        
        video_in = self.video_queue.get()
        frame = video_in.getCvFrame()
        grayscale_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        corners, ids, rejected = self.arucoDetector.detectMarkers(grayscale_frame)

        best_corners, best_id = self._get_best_tag(grayscale_frame, corners, ids)

        if best_id is not None:
            pts = best_corners[0][0]
            center_x = (float(pts[:, 0].min()) + float(pts[:, 0].max())) / 2.0
            x_offset = center_x - (self.K[0, 2])

            distance = self._calculate_distance(best_corners[0])

            return (x_offset, distance, int(best_id[0][0]))
        else:
            return None
    
    def _get_best_tag(self, frame, corners, ids):
        if ids is None or len(corners) == 0:
            return [], None
        H, W = frame.shape[:2]
        cy_img, cx_img = H / 2, W / 2

        dists = []
        for idx, c in enumerate(corners):
            corner = c[0]
            corner_x, corner_y = float(corner[:, 0].mean()), float(corner[:, 1].mean())
            dist_euclidean = (corner_x - cx_img) ** 2 + (corner_y - cy_img) ** 2

            dists.append((dist_euclidean, idx))
        
        _, best_k = min(dists, key=lambda x: x[0])

        return [corners[best_k]], np.array([[ids[best_k, 0]]], dtype=ids.dtype)
    
    def _calculate_distance(self, tag_corners):
        tag_dimensions = np.array([[-0.075, 0.075, 0.0], [0.075, 0.075, 0.0],
                                   [0.075, -0.075, 0.0], [-0.075, -0.075, 0.0]], dtype=np.float32)
        
        _, _, tVec = cv.solvePnP(tag_dimensions, tag_corners, self.K, self.DISTORTION_COEFFS)
        tVec = tVec.flatten()
        return float(np.linalg.norm([tVec[0], tVec[2]]))
    
    # --- Navigation Logic Functions ---
    def haversine_distance(self, lat1, lon1, lat2, lon2):
        """Calculate distance in meters between two GPS coordinates."""
        R = 6378137.0 # Earth radius (meters)
        lat1_r, lon1_r = math.radians(lat1), math.radians(lon1)
        lat2_r, lon2_r = math.radians(lat2), math.radians(lon2)
        
        dlat = lat2_r - lat1_r
        dlon = lon2_r - lon1_r
        
        a = math.sin(dlat/2)**2 + math.cos(lat1_r) * math.cos(lat2_r) * math.sin(dlon/2)**2
        c = 2 * math.asin(math.sqrt(a))
        return R * c

    def calculate_bearing(self, lat1, lon1, lat2, lon2):
        """Compute the compass bearing to get from pos1 to pos2."""
        lat1_r = math.radians(lat1)
        lat2_r = math.radians(lat2)
        dlon = math.radians(lon2 - lon1)
        
        y = math.sin(dlon) * math.cos(lat2_r)
        x = math.cos(lat1_r) * math.sin(lat2_r) - math.sin(lat1_r) * math.cos(lat2_r) * math.cos(dlon)
        
        bearing = math.atan2(y, x)
        return (math.degrees(bearing) + 360) % 360

    # --- Callbacks ---
    def waypoint_cb(self, msg):
        self.target_lat, self.target_lon = msg.data[0], msg.data[1] # lat, lon

    def current_pos_cb(self, msg):
        self.last_gps_time = self.get_clock().now()
        current_lat = msg.lat * 1e-7
        current_lon = msg.lon * 1e-7
        
        if self.current_lat is None:
            self.current_lat = current_lat
            self.current_lon = current_lon
        else:
            alpha = 0.9
            self.current_lat = (alpha * current_lat) + ((1.0 - alpha) * self.current_lat)
            self.current_lon = (alpha * current_lon) + ((1.0 - alpha) * self.current_lon)

    def heading_cb(self, msg):
        self.last_heading_time = self.get_clock().now()
        self.heading = msg.data

    def check_telemetry(self):
        if self.current_lat is None or self.target_lat is None or self.heading is None:
            return False
        
        now = self.get_clock().now()
        gps_age = (now - self.last_gps_time).nanoseconds / 1e9
        head_age = (now - self.last_heading_time).nanoseconds / 1e9

        if gps_age > self.TELEMETRY_TIMEOUT or head_age > self.TELEMETRY_TIMEOUT:
            self.get_logger().warn(f"Telemetry lost. GPS age: {gps_age:.2f}s, Heading age: {head_age:.2f}s", throttle_duration_sec=2.0)
            return False
        
        return True

    def stop_rover(self):
        msg = Float32MultiArray()
        msg.data = [0.0, 0.0, -1.0, -1.0]
        self.drive_pub.publish(msg)

    def control_loop(self):
        detection = self.get_latest_detection()
        
        sgnl_msg = Bool()
        sgnl_msg.data = False # Set non-detection by default

        # Base neutrals
        forward_speed = 0.0
        rot_left = -1.0
        rot_right = -1.0
        DECEL_DIST_M = 4.0

        if detection is not None:
            # --- ARUCO TAG DRIVING MODE ---
            ux, distance, tag_id = detection
            error = math.degrees(math.atan((ux)/self.K[0, 0])) # positive = right

            if distance <= 2.0:
                sgnl_msg.data = True
                # Stays neutral (forward_speed = 0.0) when within 2m
            else:
                # 1. Forward Speed Scales inversely with error
                speed_factor = max(0.2, 1.0 - (abs(error) / 45.0))
                dist_factor = min(distance / DECEL_DIST_M, 1.0)
                forward_speed = self.MAX_FORWARD_SPEED * speed_factor * dist_factor

                # 2. Turn Speed Scales proportionally with error
                turn_speed = max(min(abs(error) * self.KP_TURN, 1.0), 0.15) - 1.0
                
                # Deadband of 10 degrees to match Navigator
                if error > 10.0:
                    rot_right = turn_speed
                elif error < -10.0:
                    rot_left = turn_speed
            
        else:
            # --- GPS FALLBACK DRIVING MODE ---
            if not self.check_telemetry():
                self.stop_rover()
                self.signal_pub.publish(sgnl_msg)
                return

            dist_to_target = self.haversine_distance(
                self.current_lat, self.current_lon, 
                self.target_lat, self.target_lon
            )
            
            if dist_to_target <= 1.0:
                pass # Stays neutral 
            else:
                target_bearing = self.calculate_bearing(
                    self.current_lat, self.current_lon,
                    self.target_lat, self.target_lon
                )
                
                # Calculate heading error [-180, 180] degrees
                error = (target_bearing - self.heading + 180) % 360 - 180

                # 1. Forward Speed Scales inversely with error
                speed_factor = max(0.2, 1.0 - (abs(error) / 45.0))
                dist_factor = min(dist_to_target / DECEL_DIST_M, 1.0)
                forward_speed = self.MAX_FORWARD_SPEED * speed_factor * dist_factor

                # 2. Turn Speed Scales proportionally with error
                turn_speed = max(min(abs(error) * self.KP_TURN, 1.0), 0.15) - 1.0
                
                # Apply deadband of 10 degrees
                if error > 10.0:
                    rot_right = turn_speed
                elif error < -10.0:
                    rot_left = turn_speed

        self.signal_pub.publish(sgnl_msg)

        # Publish the command directly (Proportional Control - No Smoothing)
        cmd = [float(forward_speed), 0.0, float(rot_left), float(rot_right)]
        out_msg = Float32MultiArray()
        out_msg.data = cmd
        self.drive_pub.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraHandler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_rover()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
