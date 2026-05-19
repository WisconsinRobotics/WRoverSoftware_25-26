import cv2 as cv
import depthai as dai
import numpy as np

class CameraHandler:
    """
        This class will handle the usage of depthAI camera hardware for the purposes of both object and aruco computer vision
        This will feed into another ROS node
    """
    def __init__(self):
        self.CAMERA_WIDTH = 1280
        self.CAMERA_HEIGHT = 720

        self.K = np.array([[568.15, 0.0, 643.2372],
                           [0.0, 568.15, 367.1311],
                           [0.0, 0.0, 1.0]], dtype=np.float32)
        
        # Hopefully this works, otherwise may need to recompute the distortion coefficients
        self.DISTORTION_COEFFS = np.array([2.9425, 0.7698, -3.687e-05, 0.000175,
                                           0.00862, 3.298, 1.626, 0.0925,
                                           0.0, 0.0, 0.0, 0.0, -0.00208, 0.00402], dtype=np.float32)
        

        # Aruco defs
        self.arucoDict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
        self.arucoParams = cv.aruco.DetectorParameters()
        self.arucoDetector = cv.aruco.ArucoDetector(self.arucoDict, self.arucoParams)
        
        # Depth AI pipeline - init here or get it passed in from SM?
        self.pipeline = dai.Pipeline()
        cam = self.pipeline.create(dai.node.Camera).build()
        self.video_queue = cam.requestOutput((self.CAMERA_WIDTH, self.CAMERA_HEIGHT)).createOutputQueue()
        self.pipeline.start()

    
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
            x_offset = center_x - (self.CAMERA_WIDTH / 2.0)

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