import cv2 as cv
import depthai as dai
import numpy as np
import rclpy
import math
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
        
        # Hopefully this works, otherwise may need to recompute the distortion coefficients
        self.DISTORTION_COEFFS = np.array([2.9425, 0.7698, -3.687e-05, 0.000175,
                                           0.00862, 3.298, 1.626, 0.0925,
                                           0.0, 0.0, 0.0, 0.0, -0.00208, 0.00402], dtype=np.float32)
        

        self.waypoint_sub = self.create_subscription(Float64MultiArray, '/waypoint', self.waypoint_cb, 10)
        self.current_pos_sub = self.create_subscription(UBXNavPVT, '/rover1/ubx_nav_pvt', self.current_pos_cb, 10)
        self.heading_sub = self.create_subscription(Float32, '/heading', self.heading_cb, 10)
        self.drive_pub = self.create_publisher(Float32MultiArray, "/swerve", 10)
        self.signal_pub = self.create_publisher(Bool, "/reached_signal", 10)


        self.curr_waypoint = (None, None)
        self.curr_pos = (None, None)
        self.heading = None
        self.is_aligning = False

        # Aruco defs
        self.arucoDict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
        self.arucoParams = cv.aruco.DetectorParameters()
        self.arucoDetector = cv.aruco.ArucoDetector(self.arucoDict, self.arucoParams)
        
        # Depth AI pipeline - init here or get it passed in from SM?
        self.pipeline = dai.Pipeline()
        cam = self.pipeline.create(dai.node.Camera).build()
        self.video_queue = cam.requestOutput((self.CAMERA_WIDTH, self.CAMERA_HEIGHT)).createOutputQueue()
        self.pipeline.start()

        # Timer to run the control loop constantly (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

    
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

            # Calc distance
            distance = self._calculate_distance(best_corners[0])

            return (x_offset, distance, int(best_id[0][0]))
    
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
        # Check if these are correct
        tag_dimensions = np.array([[-0.075, 0.075, 0.0], [0.075, 0.075, 0.0],
                                   [0.075, -0.075, 0.0], [-0.075, -0.075, 0.0]], dtype=np.float32)
        
        _, _, tVec = cv.solvePnP(tag_dimensions, tag_corners, self.K, self.DISTORTION_COEFFS)
        tVec = tVec.flatten()
        return float(np.linalg.norm([tVec[0], tVec[2]]))
    

    @staticmethod
    def compute_bearing(p1, p2):
        lat1 = math.radians(p1[0]);  lon1 = math.radians(p1[1])
        lat2 = math.radians(p2[0]);  lon2 = math.radians(p2[1])
        dlon = lon2 - lon1
        x    = math.sin(dlon) * math.cos(lat2)
        y    = (math.cos(lat1) * math.sin(lat2)
                - math.sin(lat1) * math.cos(lat2) * math.cos(dlon))
        return (math.degrees(math.atan2(x, y)) + 360) % 360


    def waypoint_cb(self, msg):
        self.curr_waypoint = (msg.data[0], msg.data[1]) # lat, lon

    def current_pos_cb(self, msg):
        self.curr_pos = (msg.lat * 1e-7, msg.lon * 1e-7) # lat, lon

    def heading_cb(self, msg):
        self.heading = msg.data


    def control_loop(self):
        detection = self.get_latest_detection()
        is_tag_tracking = False
        distance = None
        error = 0.0
        sgnl_msg = Bool()

        if detection is not None:
            # We see a tag
            ux, distance, tag_id = detection
            angle_deg = math.degrees(math.atan((ux)/self.K[0, 0]))
            # positive = right
            error = angle_deg 
            is_tag_tracking = True

        elif self.curr_pos[0] is not None and self.curr_waypoint[0] is not None and self.heading is not None:
            target_bearing = self.compute_bearing(self.curr_pos, self.curr_waypoint)
            
            # Compass heading: 0=N Clockwise
            bearing_diff = target_bearing - self.heading
            
            # Normalize to [-180, 180]
            bearing_diff = (bearing_diff + 180) % 360 - 180
            
            # positive = right, negative = left
            error = bearing_diff 
        else:
            # Missing data dont move
            return
        
        if is_tag_tracking and distance is not None and distance <= 1.0:
            cmd = [0.0, 0.0, -1.0, -1.0] 
            out_msg = Float32MultiArray()
            out_msg.data = cmd
            
            sgnl_msg.data = True
            self.signal_pub.publish(sgnl_msg)
            self.drive_pub.publish(out_msg)
            return

        if self.is_aligning:
            # If we are currently turning, keep turning until we are highly accurate 
            if abs(error) <= 3.0:
                self.is_aligning = False
        else:
            # If we are driving, allow some slop. Only stop to fix it if we drift past 8°
            if abs(error) > 8.0:
                self.is_aligning = True

        KP = 0.02   
        MAX_FORWARD = 0.8
        MAX_TURN = 0.8

        
        
        if self.is_aligning:
            forward_speed = 0.0
            turn_speed = max(min(abs(error) * KP, MAX_TURN), 0.2)
        else:
            # Pure Forward Drive
            forward_speed = MAX_FORWARD
            turn_speed = -1.0

        # Positive error means the target is to the right.
        if error > 0:
            cmd = [forward_speed, 0.0, -1.0, turn_speed]  # Turn Right
        else:
            cmd = [forward_speed, 0.0, turn_speed, -1.0]  # Turn Left

        # Publish the command
        sgnl_msg.data = False
        self.signal_pub.publish(sgnl_msg)
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
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()