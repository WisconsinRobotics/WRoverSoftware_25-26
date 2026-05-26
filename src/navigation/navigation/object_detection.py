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
from std_msgs.msg import Float32MultiArray
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
        # Usage example: ros2 run object_detection object_detection --ros-args -p model:="hammer"
        self.declare_parameter('model', 'bottle')
        
        # Initialize publishers
        self.detection_publisher = self.create_publisher(Detection, 'detection_msg', 10)
        self.image_publisher = self.create_publisher(CompressedImage, 'detection_image', 10)

        self.waypoint_sub = self.create_subscription(Float64MultiArray, '/waypoint', self.waypoint_cb, 10)
        self.current_pos_sub = self.create_subscription(UBXNavPVT, '/rover1/ubx_nav_pvt', self.current_pos_cb, 10)
        self.heading_sub = self.create_subscription(Float32, '/heading', self.heading_cb, 10)
        self.drive_pub = self.create_publisher(Float32MultiArray, "/swerve", 10)
        self.signal_pub = self.create_publisher(Bool, "/reached_signal", 10)

        self.curr_waypoint = (None, None)
        self.lat = 0
        self.lon = 0

        self.origin_lat = None
        self.origin_lon = None
        self.heading = None
        self.is_aligning = False

        # Find the model path
        model_name = self.get_parameter('model').value
        model_path = os.path.join(get_package_share_directory('object_detection'), 'resource', f'{model_name}.blob')
        
        # Create pipeline
        self.pipeline = dai.Pipeline()
        
        # Camera
        cam = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A) # Create Camera node using camera A
        cam_output = cam.requestOutput((1280, 720), type=dai.ImgFrame.Type.BGR888p) # Set video resolution to 720p 3 channel BGR format
            
        # ImageManip
        manip = self.pipeline.create(dai.node.ImageManip)
        manip.initialConfig.addCrop(320, 40, 640, 640) # Crop the image to a centered 640 x 640 square
        manip.setMaxOutputFrameSize(1228800) # Set the maximum output frame size to 640 x 640 x 3
        cam_output.link(manip.inputImage) # Link camera output to manip input
        
        # NeuralNetwork
        nn = self.pipeline.create(dai.node.NeuralNetwork) # Create NeuralNetwork node
        nn.setBlobPath(model_path) # Set the .blob model path
        manip.out.link(nn.input) # Link manip output to NN input
        
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
        #config.setSubpixel(True)
        self.config.setExtendedDisparity(False)
        self.config.setLeftRightCheck(True)

        # device = pipeline.getDefaultDevice()
        # cali_data = device.readCalibration()
        # intrinsics = cali_data.getCameraIntrinsics(dai.CameraBoardSocket.CAM_B, 1280, 720)
        # print(intrinsics)

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
        
    
    def waypoint_cb(self, msg):
        self.curr_waypoint = (msg.data[0], msg.data[1]) # lat, lon

    def current_pos_cb(self, msg):
        current_lat = msg.lat * 1e-7
        current_lon = msg.lon * 1e-7
        
        if self.origin_lat is None:
            # First fix
            self.origin_lat = current_lat
            self.origin_lon = current_lon
            self.lat = current_lat
            self.lon = current_lon
        else:
            # Smooth out minor RTK jitter (Alpha = 0.6 means trust new reading 60%, old reading 40%)
            alpha = 0.80 
            self.lat = (alpha * current_lat) + ((1.0 - alpha) * self.lat)
            self.lon = (alpha * current_lon) + ((1.0 - alpha) * self.lon)

    def heading_cb(self, msg):
        self.heading = msg.data
     
        
    # Parse NN output
    def parse(self, output):
        # Flatten the array
        output = output.flatten()
        
        # Convert outputs to numpy arrays
        x = np.array(output[0:8400])
        y = np.array(output[8400:16800])
        w = np.array(output[16800:25200])
        h = np.array(output[25200:33600])
        conf = np.array(output[33600:42000])

        # Find the index of the best bounding box
        best_idx = np.argmax(conf)
        
        # Calculate the best bounding box and normalize
        x1 = (x[best_idx] - w[best_idx]/2) / 640.0
        y1 = (y[best_idx] - h[best_idx]/2) / 640.0
        x2 = (x[best_idx] + w[best_idx]/2) / 640.0
        y2 = (y[best_idx] + h[best_idx]/2) / 640.0
        
        return [x1, y1, x2, y2], conf[best_idx] # Image bounding box and confidence threshold
        
        
        
    # Calculate distance using camera focal length and bounding box height
    def calculate_distance(self, x1, y1, x2, y2):
        # Get the height of the object irl 
        if self.get_parameter('model').value == "bottle":
            real_height = 0.203
        elif self.get_parameter('model').value == "hammer":
            real_height = 0.292
        else:
            real_height = 0.3398
            
        # If your distance is consistently too SHORT, INCREASE this number.
        # If your distance is consistently too LONG, DECREASE this number.
        focal_length = 568.15

        pixel_height = max(abs(x2 - x1), abs(y2 - y1)) * 640
        
        # Distance = (real_height * focal_length) / pixel_height
        return real_height * focal_length / pixel_height
        
        
    # Publish bounding box coordinates, confidence level, and distance
    def publish_detection(self, x1, y1, x2, y2, conf, distance):
        msg = Detection()
        
        msg.x1 = float(x1)
        msg.y1 = float(y1)
        msg.x2 = float(x2)
        msg.y2 = float(y2)
        msg.conf = float(conf)
        msg.distance = float(distance)
        
        self.detection_publisher.publish(msg)
                
    # Publish image with bounding box
    def publish_image(self, x1, y1, x2, y2, conf, distance):
        # Camera data
        cam_data = self.cam.tryGet()
        
        if cam_data is not None:
            # Get frame
            frame = cam_data.getCvFrame()
            if conf > 0.5:
            # Draw bounding boxes
                x1 = int(x1 * 640)
                y1 = int(y1 * 640)
                x2 = int(x2 * 640)
                y2 = int(y2 * 640)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f"Confidence: {conf:.2f}", (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(frame, f"Distance: {distance:.2f}m", (x1, y1-30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Convert to CompressedImage and publish
            image = CompressedImage()
            image.header.stamp = self.get_clock().now().to_msg()
            image.format = "jpeg"
            image.data = np.array(cv2.imencode('.jpg', frame)[1]).tobytes()
            self.image_publisher.publish(image)
            


    def get_latest_detection(self):
        """
            Polls the neural network and returns an (offset_x, distance, id) of the detected best object, or None
        """
        nn_data = self.nn.tryGet()
        
        if nn_data is not None:
            output = nn_data.getTensor("output0")
            box, conf = self.parse(output)
            
            if conf > 0.5:
                x1, y1, x2, y2 = box
                distance = self.calculate_distance(x1, y1, x2, y2)

                # Publish detection msg
                self.publish_detection(x1, y1, x2, y2, conf, distance)
                
                # Publish image msg
                self.publish_image(x1, y1, x2, y2, conf, distance)
                
                # Center of the bounding box in normalized coordinates [0, 1]
                x_center_norm = (x1 + x2) / 2.0
                
                # Map to cropped 640x640 frame scale
                x_center_cropped = x_center_norm * 640.0
                
                # Map back to original 1280x720 frame (which was center-cropped starting from x=320)
                x_center_original = x_center_cropped + 320.0
                
                # Calculate x pixel offset from camera optical center (cx)
                ux = x_center_original - self.K[0, 2]
                
                return (ux, distance, None) # return None for tag_id, as it isn't utilized by the control loop
                
        return None
    
    def control(self):
        if self.heading is None:
            return
        detection = self.get_latest_detection()
        is_tag_tracking = False
        distance = None
        error = 0.0
        sgnl_msg = Bool()
        out_msg = Float32MultiArray()

        if detection is not None:
            # do the tank drive thing
            ux, distance, tag_id = detection
            angle_deg = math.degrees(math.atan((ux)/self.K[0, 0]))
            # positive = right
            error = angle_deg
            is_tag_tracking = True

            # write the swerve commands in here cause the rest of the block controls the obstacle avoidance swerve commands

            if is_tag_tracking and distance is not None and distance <= 2.0:
                cmd = [0.0, 0.0, -1.0, -1.0] 
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

            out_msg.data = cmd
            self.drive_pub.publish(out_msg)

        else:
            stereoFrame = self.stereoOut.tryGet()
            self.obstacle_avoider.lat        = self.lat
            self.obstacle_avoider.lon        = self.lon
            self.obstacle_avoider.origin_lat = self.origin_lat
            self.obstacle_avoider.origin_lon = self.origin_lon
            self.obstacle_avoider.waypoint = self.curr_waypoint
            self.obstacle_avoider.heading = self.heading
            if stereoFrame is None:
                return
            self.depth      = stereoFrame.getCvFrame().astype(np.float32) / 1000.0
            swerve_cmd = Float32MultiArray()
            swerve_cmd.data = self.obstacle_avoider.cb(self.depth, self.heading)
            self.drive_pub.publish(swerve_cmd)
                
            
def main(args=None):
    rclpy.init(args=args)
    
    publisher = ObjectDetection()
    
    rclpy.spin(publisher)
    
    publisher.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()