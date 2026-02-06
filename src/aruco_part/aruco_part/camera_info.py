#!/usr/bin/env python3

import numpy as np
import cv2
import cv_bridge
import depthai as dai
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


def aruco_display(corners, ids, rejected, image):
    if len(corners) > 0:

        ids = ids.flatten()

        for (markerCorner, markerID,) in zip(corners, ids):
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners

            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))

            cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
            cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)

            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)

            cv2.putText(image, str(markerID), (topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_PLAIN,
                        0.5, (0, 255, 0), 2)

            print("[Interference] ArUco marker ID: {}".format(markerID))

    return image


class CameraInfoPublisher(Node):
    def __init__(self):
        super().__init__("camera_info")
        CAMERA_WIDTH = 1280
        CAMERA_HEIGHT = 720

        # Initialize camera
        self.pipeline = dai.Pipeline()
        cam = self.pipeline.create(dai.node.Camera).build()
        self.videoQueue = cam.requestOutput(
            (CAMERA_WIDTH, CAMERA_HEIGHT)
        ).createOutputQueue()
        self.pipeline.start()

        self.cam_info_pub = self.create_publisher(Image, "camera_data_topic", 10)
        self.timer_ = self.create_timer(0.01,
                                        self.cameraCallback)  # camera: 60fps = 0.0167 sec/frame. So, round to 0.02

    def cameraCallback(self):  # this is the callback
        arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)  # We want the 4x4 50 dictionary
        arucoParams = cv2.aruco.DetectorParameters()

        videoIn = self.videoQueue.get()
        assert isinstance(videoIn, dai.ImgFrame)
        frame = videoIn.getCvFrame()  # pass as the img param to the detect aruco function
        corners, ids, rejected = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)
        detected_markers = aruco_display(corners, ids, rejected, frame)

        msg = self.bridge.cv2_to_imgmsg(detected_markers, encoding='bgr8')
        self.cam_info_pub.publish(msg)
        self.get_logger().info("Publishing frame...")


def main(args=None):
    rclpy.init(args=args)
    camera_info_publisher = CameraInfoPublisher()

    rclpy.spin(camera_info_publisher)

    camera_info_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()