import os
import cv2
import rclpy
import numpy as np
import depthai as dai
from rclpy.node import Node
from object_detection_msgs.msg import Detection
from ament_index_python.packages import get_package_share_directory



class HammerDetector(Node):
    def __init__(self):
        # --- Initialize publisher ---
        super().__init__('hammer_detector')
        self.publisher_ = self.create_publisher(Detection, 'hammer_detections', 10)
        
        # --- Find the model path ---
        model_path = os.path.join(get_package_share_directory('object_detection'), 'resource', 'hammer.blob')
        
        # --- Create pipeline ---
        self.pipeline = dai.Pipeline()
        
        # --- Camera ---
        cam = self.pipeline.create(dai.node.Camera).build() # Create Camera node
        cam_output = cam.requestOutput((1280, 720), type=dai.ImgFrame.Type.BGR888p) # Set video resolution to 720p 3 channel BGR format
        
        # --- ImageManip ---
        manip = self.pipeline.create(dai.node.ImageManip) # Create ImageManip node
        manip.initialConfig.addCrop(320, 40, 640, 640) # Crop the image to a centered 640 x 640 square
        manip.setMaxOutputFrameSize(1228800) # Set the maximum output frame size to 640 x 640 x 3
        cam_output.link(manip.inputImage) # Link camera output to manip input
        
        # --- NeuralNetwork ---
        nn = self.pipeline.create(dai.node.NeuralNetwork) # Create NeuralNetwork node
        nn.setBlobPath(model_path) # Set the .blob model path
        manip.out.link(nn.input) # Link manip output to NN input
        
        # --- NN and camera output queues ---
        self.nn = nn.out.createOutputQueue(maxSize=4, blocking=False)
        self.cam = manip.out.createOutputQueue(maxSize=4, blocking=False)
        
        # --- Start pipeline ---
        self.pipeline.start()
        
        # --- Timer that checks nn output 30 times a second ---
        self.timer = self.create_timer(0.033, self.check_output)
        
    # --- Parse output ---
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
        
        return [x1, y1, x2, y2], conf[best_idx]
        
    # --- Check output ---
    def check_output(self):
        # NN data
        nn_data = self.nn.tryGet()
        
        # Camera data
        cam_data = self.cam.tryGet()
        if cam_data is not None:
            frame = cam_data.getCvFrame()
        
        if nn_data is not None:
            # Get output layer
            output = nn_data.getTensor("output0")
            
            # Parse the outputs
            best_box, best_conf = self.parse(output)
            
            # If valid detection, publish custom detection msg
            if best_conf > 0.5:
                msg = Detection()
                
                msg.x1 = float(best_box[0])
                msg.y1 = float(best_box[1])
                msg.x2 = float(best_box[2])
                msg.y2 = float(best_box[3])
                msg.conf = float(best_conf)
                msg.distance = 0.0
                
                self.publisher_.publish(msg) 

            

def main(args=None):
    rclpy.init(args=args)
    
    publisher = HammerDetector()
    
    rclpy.spin(publisher)
    
    publisher.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
