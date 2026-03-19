import os
import cv2
import rclpy
import numpy as np
import depthai as dai
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from object_detection_msgs.msg import Detection
from ament_index_python.packages import get_package_share_directory



class ObjectDetection(Node):
    def __init__(self):
        # Initialize node
        super().__init__('object_detection')
        
        # Declare parameters
        # Usage example: ros2 run object_detection object_detection --ros-args -p model:="hammer"
        self.declare_parameter('model', 'bottle')
        
        # Initialize publishers
        self.detection_publisher = self.create_publisher(Detection, 'detection_msg', 10)
        self.image_publisher = self.create_publisher(Image, 'detection_image', 10)
        self.swerve_publisher = self.create_publisher(Float32MultiArray, 'swerve', 10)
        self.led_publisher = self.create_publisher(Float32MultiArray, 'led', 1)
        
        # Initialize LED
        self.flashing = True
        self.publish_led("RED")
        
        # Initialize cvbridge
        self.bridge = CvBridge()
        
        # Find the model path
        model_path = os.path.join(get_package_share_directory('object_detection'), 'resource', self.get_parameter('model').value + '.blob')
        
        # Create pipeline
        self.pipeline = dai.Pipeline()
        
        # Camera
        cam = self.pipeline.create(dai.node.Camera).build() # Create Camera node
        cam_output = cam.requestOutput((1280, 720), type=dai.ImgFrame.Type.BGR888p) # Set video resolution to 720p 3 channel BGR format
        
        # ImageManip
        manip = self.pipeline.create(dai.node.ImageManip) # Create ImageManip node
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
        
        # Start pipeline
        self.pipeline.start()
        
        # Timer that checks nn output at 30 Hz
        self.timer = self.create_timer(0.033, self.publish)
        
     
        
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
        
        return [x1, y1, x2, y2], conf[best_idx], x[best_idx]
        
        
        
    # Calculate bottle distance using camera focal length and bounding box height
    # TODO: replace with pixel-based depth calculation
    def calculate_bottle_distance(self, y1, y2):
        REAL_BOTTLE_HEIGHT = 0.20 #in meters
        
        # Approx 930 for OAK-D @ 1280x720 resolution
        # If your distance is consistently too SHORT, INCREASE this number.
        # If your distance is consistently too LONG, DECREASE this number.
        FOCAL_LENGTH_PIXELS = 931.0 
        
        # The crop is 640x640, but the pixels are 1:1 from the 1280x720 feed.
        # So we use the frame_height of the CROP (640) to convert normalized coords.
        FRAME_HEIGHT_PIXELS = 640.0 

        # --- CALCULATION ---
        # 1. Get height of box in pixels (y1, y2 are 0.0 to 1.0)
        pixel_height = abs(y2 - y1) * FRAME_HEIGHT_PIXELS
        
        # 2. Prevent division by zero errors
        if pixel_height < 5.0: 
            return 0.0
        
        # 3. Pinhole Camera Formula
        # Distance = (Real_Size * Focal_Length) / Pixel_Size
        dist = (REAL_BOTTLE_HEIGHT * FOCAL_LENGTH_PIXELS) / pixel_height
        
        return dist
        
        
        
    # Calculate swerve velocities based on detection position
    def calculate_vel(self, x_offset, distance, linear_velocity):
        x_center = 320 # Center of the 640 crop
        angular_vel = 0.0
        current_linear = linear_velocity
        
        # Slow down as we get closer
        # TODO: test distance thresholds
        if 1.0 < distance < 2.5:
            current_linear = linear_velocity / 2
        elif distance <= 1.0:
            current_linear = 0.0

        max_error = 320 # Half width of crop
        error = x_offset - x_center
        
        # Deadband logic
        if abs(error) > 20:
            angular_vel = 1.0 * (error / max_error)
            # If extremely offset, stop moving forward and just rotate
            if abs(error) > 200:
                current_linear = 0.0
                
        return current_linear, angular_vel
        
        
        
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
            
            # Draw bounding boxes
            x1 = int(x1 * 640)
            y1 = int(y1 * 640)
            x2 = int(x2 * 640)
            y2 = int(y2 * 640)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"Confidence: {conf:.2f}", (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(frame, f"Distance: {distance:.2f}m", (x1, y1-30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Convert to image message and publish
            image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.image_publisher.publish(image)
            
            
            
    # Publish LED color info
    def publish_led(self, color):
        led_msg = Float32MultiArray()
        self.flashing = not self.flashing
        
        if color == "RED":
            led_msg.data = [255.0, 0.0, 0.0]
        elif color == "GREEN":
            # Blink logic
            # TODO: test blinking rate (currently 30Hz)
            led_msg.data = [0.0, 255.0, 0.0] if self.flashing else [0.0, 0.0, 0.0]
        elif color == "BLUE":
            led_msg.data = [0.0, 0.0, 255.0]
        else:
            led_msg.data = [0.0, 0.0, 0.0]
        
        self.led_publisher.publish(led_msg)
        
        
        
    # Publish swerve control parameters linear_y, linear_x, angular_pos, and angular_neg
    def publish_swerve(self, linear_y, linear_x, angular_pos, angular_neg):
        swerve_msg = Float32MultiArray()
        
        swerve_msg.data = [linear_y, linear_x, angular_pos, angular_neg]
        
        self.swerve_publisher.publish(swerve_msg)



    def publish(self):
        # NN data
        nn_data = self.nn.tryGet()
        
        if nn_data is not None:
            # Get output layer
            output = nn_data.getTensor("output0")
            
            # Parse the outputs
            box, conf, center_x = self.parse(output)
            
            # If valid detection
            if conf > 0.5:
                # TODO: replace with generalized pixel-based distance calculation
                distance = self.calculate_bottle_distance(box[1], box[3])
                
                # Calculate linear and angular velocity
                linear_vel, angular_vel = self.calculate_vel(center_x, distance, 1.5)
                
                # Publish detection msg
                self.publish_detection(box[0], box[1], box[2], box[3], conf, distance)
                
                # Publish image msg
                self.publish_image(box[0], box[1], box[2], box[3], conf, distance)
                
                # Publish swerve msg
                self.publish_swerve(float(linear_vel), 0.0, float(-angular_vel), float(angular_vel))
                
                # Publish LED msg
                if linear_vel == 0.0:
                    self.publish_led("GREEN") # Goal reached
                else:
                    self.publish_led("BLUE") # Tracking
            else:
                # Publish 0s as detection msg
                self.publish_detection(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                
                # Rotote to search and set LED to red
                # TODO: test rotation speed
                # TODO: implement search algorithm
                self.publish_swerve(0.0, 0.0, -1.0, 1.0)
                self.publish_led("RED")
                
                
            
def main(args=None):
    rclpy.init(args=args)
    
    publisher = ObjectDetection()
    
    rclpy.spin(publisher)
    
    publisher.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
