#!/usr/bin/env python3

import numpy as np
import cv2
import rclpy
from numpy.matlib import empty
from rclpy.node import Node
import cv_bridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720

K = [[569.7166137695312, 0.0, 622.4732666015625], [0.0, 569.48486328125, 367.3853454589844], [0.0, 0.0, 1.0]]

DISTORSION_COEFFS = [2.9425814151763916, 0.7698521018028259, -3.687290518428199e-05, 0.00017509849567431957,
                     0.00862385705113411, 3.298475503921509, 1.6266201734542847, 0.09254294633865356,
                     0.0, 0.0, 0.0, 0.0, -0.0020870917942374945, 0.004025725182145834]

K = np.array(K, dtype=np.float32)  # 3x3
DISTORSION_COEFFS = np.array(DISTORSION_COEFFS, dtype=np.float32)


def aruco_display(corners, ids, rejected, image):
    if len(corners) > 0:

        ids = ids.flatten()

        for(markerCorner, markerID,) in zip(corners, ids):

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
                        0.5, (0,255,0), 2)


            print("[Interference] ArUco marker ID: {}".format(markerID))


    return image


'''This function picks picks the tag that is closer to the center of the frame.
This way we don't have to deal with 2 tags of the same ID and just focus on 1
side of the post.'''
def getBestTag(frame, corners, ids):
    if ids is None or len(corners) == 0:
        return [], None

    H, W = frame.shape[:2]
    cy_img, cx_img = H/2, W/2

    # Build list of (squared_distance, index)
    dists = []
    for k, c in enumerate(corners):
        crn = c[0]  # (4,2)
        cx = float(crn[:, 0].mean())
        cy = float(crn[:, 1].mean())
        d2 = (cx - cx_img) ** 2 + (cy - cy_img) ** 2
        dists.append((d2, k))

    # Pick the min-distance detection
    _, best_k = min(dists, key=lambda x: x[0])
    best_corner = corners[best_k]
    best_id = int(ids[best_k, 0])

    return [best_corner], np.array([[best_id]], dtype=ids.dtype)


def calculateDistance(tag_corners, K_matrix, dist_coefficients):  #MAKE K AND DIST COEFFS CONSTANTS INSIDE THE FUNCTION
    # coordinate of the tag considering real world dimensions
    tag_dimensions = np.array([[-0.075, 0.075, 0.0],
                               [0.075, 0.075, 0.0],
                               [0.075, -0.075, 0.0],
                               [-0.075, -0.075, 0.0]],
                              dtype=np.float32)

    # tag_corners = tag_corners[0]

    # Using homogeneous coordinates to get translation and rotation vectors
    # print(f"COrners shape: {tag_corners}, dimensionns shape: {tag_dimensions.shape}")
    retVal, rVec, tVec = cv2.solvePnP(tag_dimensions, tag_corners, K_matrix, dist_coefficients)

    tVec = tVec.flatten()

    tag_distance = float(np.linalg.norm([tVec[0], tVec[2]])) #distance from the center of the tag in meters. Ignore y

    return tag_distance



def detect_aruco(img: np.ndarray):
    """Detects ArUco markers

    @param img (np.ndarray): An OpenCV image
    @return Tuple[np.ndarray, np.ndarray, np.ndarray]: A tuple containing the corners, ids, and rejected points in the image
    """
    arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)  # We want the 4x4 50 dictionary
    arucoParams = cv2.aruco.DetectorParameters()

    corners, ids, rejected = cv2.aruco.detectMarkers(img, arucoDict, parameters=arucoParams)
    corners, ids = getBestTag(img, corners, ids)  # Will make the program display just the best tag

    return corners, ids, rejected


class ArucoDetectionPublisher(Node):

    def __init__(self):
        super().__init__('aruco_detection')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'aruco_results', 10)
        self.bridge = cv_bridge.CvBridge()
        self.subscription = self.create_subscription(
            Image,
            'camera_data_topic',
            self.image_callback,
            10)
        self.tags_detected = []
        self.tags_fs_detected = []

        self.tags = {}

        #self.subscription  # prevent unused variable warning

    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        (corners, ids, _) = detect_aruco(img)
        pub_msg = Float32MultiArray()
        if ids is not None:
            for i, marker_id in enumerate(ids):
                # corners[i]: shape (1, 4, 2)
                pts = corners[i][0]  # shape (4, 2)
                xs = pts[:, 0]  # all x coordinates of the 4 corners

                min_x = float(xs.min())
                max_x = float(xs.max())

                # center-of-tag x in pixels
                tag_center_x = (min_x + max_x) / 2.0
                image_center_x = CAMERA_WIDTH / 2.0
                x_offset = tag_center_x - image_center_x

                # Estimate the distance of the ArUco tag in meters
                distance_estimate = calculateDistance(corners[0], K, DISTORSION_COEFFS)

                id_num = int(ids[i][0])

                self.tags[f"{id_num}"] = [int(x_offset), float(distance_estimate)]

                if self.tags_detected.count(id_num) < 12:
                    self.tags_detected.append(id_num)

                for j in range(50):
                    if self.tags_detected.count(j) > 11 and j not in self.tags_fs_detected:
                        print("Aruco tag with ID " + str(j) + " for sure detected.")
                        self.tags_fs_detected.append(j)

                if len(self.tags_fs_detected) > 0:
                    target_id = self.tags_fs_detected[0]
                    x = float (self.tags[f"{target_id}"][0])
                    dis = float (self.tags[f"{target_id}"][1])
                    pub_msg.data = [float (target_id), x, dis]

                    self.publisher_.publish(pub_msg)
                    self.get_logger().info(f"Publishing {pub_msg.data[0]}, {pub_msg.data[1]}, {pub_msg.data[2]}")
                else:
                    target_id = -1.0
                    x = 0.0
                    dis = 0.0
                    pub_msg.data = [target_id, x, dis]

                    self.get_logger().info(f"No aruco tags detected")
                    self.publisher_.publish(pub_msg)
        else:
            target_id = -1.0
            x = 0.0
            dis = 0.0
            pub_msg.data = [target_id, x, dis]

            self.get_logger().info(f"No aruco tags detected")
            self.publisher_.publish(pub_msg)


def main(args=None):
    rclpy.init(args=args)
    aruco_detection_publisher = ArucoDetectionPublisher()

    rclpy.spin(aruco_detection_publisher)

    aruco_detection_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()