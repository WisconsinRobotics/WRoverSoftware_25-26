import os
import cv2
import numpy as np
import depthai as dai

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import CompressedImage
from object_detection_msgs.msg import Detection
from ament_index_python.packages import get_package_share_directory
from navigation.obstacle_avoider import SectorDepthClassifier
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float32
from ublox_ubx_msgs.msg import UBXNavPVT
from std_msgs.msg import Bool
import math

class ObjectDetection(Node):
    def __init__(self):
        # Initialize node
        super().__init__('object_detection')
        
        self.K = np.array([[568.15, 0.0, 643.2372],
                           [0.0, 568.15, 367.1311],
                           [0.0, 0.0, 1.0]], dtype=np.float32)
                           
        # Declare parameters
        self.declare_parameter('model', 'bottle')
        
        # Initialize publishers/subscribers
        self.detection_publisher = self.create_publisher(Detection, 'detection_msg', 10)
        self.image_publisher = self.create_publisher(CompressedImage, 'detection_image', 10)

        self.waypoint_sub = self.create_subscription(Float64MultiArray, '/waypoint', self.waypoint_cb, 10)
        self.current_pos_sub = self.create_subscription(UBXNavPVT, '/rover1/ubx_nav_pvt', self.current_pos_cb, 10)
        self.heading_sub = self.create_subscription(Float32, '/heading', self.heading_cb, 10)
        
        self.drive_pub = self.create_publisher(Float32MultiArray, "/swerve", 10)
        self.signal_pub = self.create_publisher(Bool, "/reached_signal", 10)

        # Matched properties from Navigator Node
        self.MAX_FORWARD_SPEED = 0.8 
        self.KP_TURN = 0.015
        self.TELEMETRY_TIMEOUT = 5.0 # Seconds

        self.curr_waypoint = (None, None)
        self.current_lat = None
        self.current_lon = None
        self.origin_lat = None
        self.origin_lon = None
        self.heading = None

        self.last_gps_time = self.get_clock().now()
        self.last_heading_time = self.get_clock().now()

        # Find the model path
        model_name = self.get_parameter('model').value
        model_path = os.path.join(get_package_share_directory('navigation'), 'resource', f'{model_name}.blob')
        
        # Create pipeline
        self.pipeline = dai.Pipeline()
        
        # Camera
        cam = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        cam_output = cam.requestOutput((1280, 720), type=dai.ImgFrame.Type.BGR888p)
            
        # ImageManip
        manip = self.pipeline.create(dai.node.ImageManip)
        manip.initialConfig.addCrop(320, 40, 640, 640)
        manip.setMaxOutputFrameSize(1228800)
        cam_output.link(manip.inputImage)
        
        # NeuralNetwork
        nn = self.pipeline.create(dai.node.NeuralNetwork)
        nn.setBlobPath(model_path)
        manip.out.link(nn.input)
        
        # NN and camera output queues
        self.nn = nn.out.createOutputQueue(maxSize=4, blocking=False)
        self.cam = manip.out.createOutputQueue(maxSize=4, blocking=False)

        # obstacle avoidance pipeline shit
        self.monoLeft  = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
        self.monoRight = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
        self.stereo    = self.pipeline.create(dai.node.StereoDepth)

        self.stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.ROBOTICS)
        self.stereo.setDepthAlign(dai.CameraBoardSocket.CAM_B)
        self.stereo.setOutputSize(1280, 720)

        self.config = self.stereo.initialConfig
        self.config.postProcessing.median                   = dai.MedianFilter.KERNEL_5x5
        self.config.postProcessing.thresholdFilter.maxRange = 7500
        self.config.setConfidenceThreshold(50)
        self.config.setExtendedDisparity(False)
        self.config.setLeftRightCheck(True)

        self.monoLeftOut  = self.monoLeft.requestOutput((1280, 720))
        self.monoRightOut = self.monoRight.requestOutput((1280, 720))
        self.monoLeftOut.link(self.stereo.left)
        self.monoRightOut.link(self.stereo.right)

        self.stereoOut = self.stereo.depth.createOutputQueue(maxSize=1, blocking=False)

        self.obstacle_avoider = SectorDepthClassifier()
        
        # Start pipeline
        self.pipeline.start()
        
        # Timer that checks nn output at 30 Hz
        self.timer = self.create_timer(0.033, self.control)
        self.get_logger().info("Object Detection Node initialized (Proportional Mode)...")
        
    # --- Navigation Logic Functions / Callbacks ---
    def waypoint_cb(self, msg):
        self.curr_waypoint = (msg.data[0], msg.data[1]) # lat, lon

    def current_pos_cb(self, msg):
        self.last_gps_time = self.get_clock().now()
        current_lat = msg.lat * 1e-7
        current_lon = msg.lon * 1e-7
        
        if self.current_lat is None:
            # First fix
            self.origin_lat = current_lat
            self.origin_lon = current_lon
            self.current_lat = current_lat
            self.current_lon = current_lon
        else:
            # Alpha 0.9 matches Navigator logic
            alpha = 0.9
            self.current_lat = (alpha * current_lat) + ((1.0 - alpha) * self.current_lat)
            self.current_lon = (alpha * current_lon) + ((1.0 - alpha) * self.current_lon)

    def heading_cb(self, msg):
        self.last_heading_time = self.get_clock().now()
        self.heading = msg.data

    def check_telemetry(self):
        if self.current_lat is None or self.curr_waypoint[0] is None or self.heading is None:
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
        
    # --- NN & CV Logic ---
    def parse(self, output):
        output = output.flatten()
        
        x = np.array(output[0:8400])
        y = np.array(output[8400:16800])
        w = np.array(output[16800:25200])
        h = np.array(output[25200:33600])
        conf = np.array(output[33600:42000])

        best_idx = np.argmax(conf)
        
        x1 = (x[best_idx] - w[best_idx]/2) / 640.0
        y1 = (y[best_idx] - h[best_idx]/2) / 640.0
        x2 = (x[best_idx] + w[best_idx]/2) / 640.0
        y2 = (y[best_idx] + h[best_idx]/2) / 640.0
        
        return [x1, y1, x2, y2], conf[best_idx]
        
    def calculate_distance(self, x1, y1, x2, y2):
        if self.get_parameter('model').value == "bottle":
            real_height = 0.203
        elif self.get_parameter('model').value == "hammer":
            real_height = 0.292
        else:
            real_height = 0.3398
            
        focal_length = 568.15
        pixel_height = max(abs(x2 - x1), abs(y2 - y1)) * 640
        return real_height * focal_length / pixel_height
        
    def publish_detection(self, x1, y1, x2, y2, conf, distance):
        msg = Detection()
        msg.x1 = float(x1)
        msg.y1 = float(y1)
        msg.x2 = float(x2)
        msg.y2 = float(y2)
        msg.conf = float(conf)
        msg.distance = float(distance)
        self.detection_publisher.publish(msg)
                
    def publish_image(self, x1, y1, x2, y2, conf, distance):
        cam_data = self.cam.tryGet()
        
        if cam_data is not None:
            frame = cam_data.getCvFrame()
            if conf > 0.5:
                x1 = int(x1 * 640)
                y1 = int(y1 * 640)
                x2 = int(x2 * 640)
                y2 = int(y2 * 640)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f"Confidence: {conf:.2f}", (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(frame, f"Distance: {distance:.2f}m", (x1, y1-30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            image = CompressedImage()
            image.header.stamp = self.get_clock().now().to_msg()
            image.format = "jpeg"
            image.data = np.array(cv2.imencode('.jpg', frame)[1]).tobytes()
            self.image_publisher.publish(image)
            
    def get_latest_detection(self):
        nn_data = self.nn.tryGet()
        
        if nn_data is not None:
            output = nn_data.getTensor("output0")
            box, conf = self.parse(output)
            
            if conf > 0.5:
                x1, y1, x2, y2 = box
                distance = self.calculate_distance(x1, y1, x2, y2)

                self.publish_detection(x1, y1, x2, y2, conf, distance)
                self.publish_image(x1, y1, x2, y2, conf, distance)
                
                x_center_norm = (x1 + x2) / 2.0
                x_center_cropped = x_center_norm * 640.0
                x_center_original = x_center_cropped + 320.0
                ux = x_center_original - self.K[0, 2]
                
                return (ux, distance, None)
                
        return None
    
    # --- Main Control Loop ---
    def control(self):
        detection = self.get_latest_detection()
        
        sgnl_msg = Bool()
        sgnl_msg.data = False
        out_msg = Float32MultiArray()

        # Base neutrals
        forward_speed = 0.0
        rot_left = -1.0
        rot_right = -1.0
        DECEL_DIST_M = 4.0

        if detection is not None:
            # --- OBJECT TRACKING MODE ---
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
                
                # Apply deadband of 10 degrees to match Navigator
                if error > 10.0:
                    rot_right = turn_speed
                elif error < -10.0:
                    rot_left = turn_speed

            self.signal_pub.publish(sgnl_msg)
            
            # Publish proportional command
            cmd = [float(forward_speed), 0.0, float(rot_left), float(rot_right)]
            out_msg.data = cmd
            self.drive_pub.publish(out_msg)

        else:
            # --- OBSTACLE AVOIDANCE (FALLBACK) MODE ---
            if not self.check_telemetry():
                self.stop_rover()
                self.signal_pub.publish(sgnl_msg)
                return
                
            stereoFrame = self.stereoOut.tryGet()
            if stereoFrame is None:
                return

            self.obstacle_avoider.lat        = self.current_lat
            self.obstacle_avoider.lon        = self.current_lon
            self.obstacle_avoider.origin_lat = self.origin_lat
            self.obstacle_avoider.origin_lon = self.origin_lon
            self.obstacle_avoider.waypoint   = self.curr_waypoint
            self.obstacle_avoider.heading    = self.heading
            
            self.depth = stereoFrame.getCvFrame().astype(np.float32) / 1000.0
            
            swerve_cmd = Float32MultiArray()
            swerve_cmd.data = self.obstacle_avoider.cb(self.depth, self.heading)
            self.drive_pub.publish(swerve_cmd)
            self.signal_pub.publish(sgnl_msg)
                
            
def main(args=None):
    rclpy.init(args=args)
    publisher = ObjectDetection()
    
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.stop_rover()
        publisher.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
