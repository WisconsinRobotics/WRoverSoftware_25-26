import numpy as np
import depthai as dai

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from navigation.obstacle_avoider import SectorDepthClassifier
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from ublox_ubx_msgs.msg import UBXNavPVT



class ObjectDetection(Node):
    def __init__(self):
        # Initialize node
        super().__init__('object_detection')
        
        self.K = np.array([[568.15, 0.0, 643.2372],
                           [0.0, 568.15, 367.1311],
                           [0.0, 0.0, 1.0]], dtype=np.float32)

        self.waypoint_sub = self.create_subscription(Float64MultiArray, '/waypoint', self.waypoint_cb, 10)
        self.current_pos_sub = self.create_subscription(UBXNavPVT, '/rover1/ubx_nav_pvt', self.current_pos_cb, 10)
        self.heading_sub = self.create_subscription(Float32, '/heading', self.heading_cb, 10)
        self.drive_pub = self.create_publisher(Float32MultiArray, "/swerve", 10)

        self.curr_waypoint = (None, None)
        self.lat = 0
        self.lon = 0

        self.origin_lat = None
        self.origin_lon = None
        self.heading = None

        # Create pipeline
        self.pipeline = dai.Pipeline()
        
        # obstacle avoidance pipeline shit

        self.monoLeft  = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
        self.monoRight = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
        self.stereo    = self.pipeline.create(dai.node.StereoDepth)

        self.stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.ROBOTICS)
        self.stereo.setDepthAlign(dai.CameraBoardSocket.CAM_B)
        self.stereo.setOutputSize(1280, 720)

        self.config = self.stereo.initialConfig
        self.config.postProcessing.median                   = dai.MedianFilter.KERNEL_7x7
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
        
        # controls output at 33 hz
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
     

    def control(self):
        if self.heading is None or self.curr_waypoint is (None, None):
            return

        stereoFrame = self.stereoOut.tryGet()
        self.obstacle_avoider.lat        = self.lat
        self.obstacle_avoider.lon        = self.lon
        self.obstacle_avoider.origin_lat = self.origin_lat
        self.obstacle_avoider.origin_lon = self.origin_lon
        self.obstacle_avoider.waypoint = self.curr_waypoint
        self.obstacle_avoider.heading = self.heading
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
