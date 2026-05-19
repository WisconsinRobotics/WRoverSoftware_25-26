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



class ObjectDetection(Node):
    def __init__(self):
        # Initialize node
        super().__init__('object_detection')
        
        # Declare parameters
        # Usage example: ros2 run object_detection object_detection --ros-args -p model:="hammer"
        self.declare_parameter('model', 'bottle')
        
        # Initialize publishers
        self.detection_publisher = self.create_publisher(Detection, 'detection_msg', 10)
        self.image_publisher = self.create_publisher(CompressedImage, 'detection_image', 10)

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
        
        return [x1, y1, x2, y2], conf[best_idx] # Image bounding box and confidence threshold
        
        
        
    # Calculate distance using camera focal length and bounding box height
    def calculate_distance(self, x1, y1, x2, y2):
        # Get the height of the object irl TODO - check if heights are accurate
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
            

    def publish(self):
        # NN data
        nn_data = self.nn.tryGet()
        
        if nn_data is not None:
            # Get output layer
            output = nn_data.getTensor("output0")
            
            # Parse the outputs
            box, conf = self.parse(output)
            
            # If valid detection
            if conf > 0.5:
                # Calculate the distance
                distance = self.calculate_distance(box[0], box[1], box[2], box[3])
                
                # Calculate linear and angular velocity
                # linear_vel, angular_vel = self.calculate_vel(center_x, distance, 1.5)
                
                # Publish detection msg
                self.publish_detection(box[0], box[1], box[2], box[3], conf, distance)
                
                # Publish image msg
                self.publish_image(box[0], box[1], box[2], box[3], conf, distance)
                
            else:
                # Publish 0s as detection msg
                self.publish_detection(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                self.publish_image(None, None, None, None, 0.0, 0.0) # Publish image even if no object detected
                
            
def main(args=None):
    rclpy.init(args=args)
    
    publisher = ObjectDetection()
    
    rclpy.spin(publisher)
    
    publisher.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()