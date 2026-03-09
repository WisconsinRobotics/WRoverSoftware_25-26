import json
import threading
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

KEYBOARD_WIDTH_MM = 354.0
KEYBOARD_DEPTH_MM = 123.0
PITCH = 19.05
H_KEY = PITCH

ROW_Y = {'fn': 6.5, 'num': 25.55, 'tab': 44.60, 'caps': 63.65, 'shft': 82.70, 'spc': 101.75}

ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
ARUCO_PARAMS = cv2.aruco.DetectorParameters()

DST_PTS = np.array([
    [0.0, 0.0],
    [KEYBOARD_WIDTH_MM, 0.0],
    [KEYBOARD_WIDTH_MM, KEYBOARD_DEPTH_MM],
    [0.0, KEYBOARD_DEPTH_MM],
], dtype=np.float32)

KEY_LAYOUT = []

def u(n):
    return n * PITCH

def add_row(keys, y, start_x):
    x = start_x
    for label, w in keys:
        w_mm = u(w)
        KEY_LAYOUT.append((label, x + w_mm / 2, y, w_mm))
        x += w_mm

add_row([('`',1),('1',1),('2',1),('3',1),('4',1),('5',1),('6',1),('7',1),('8',1),('9',1),('0',1),('-',1),('=',1),('Bksp',2)], ROW_Y['num'], 4.5)

KEY_MAP = {lbl: {'cx_mm': cx, 'cy_mm': cy, 'w_mm': w, 'h_mm': H_KEY} for lbl, cx, cy, w in KEY_LAYOUT}

def marker_center(corners_one):
    pts = corners_one.reshape((4,2))
    return np.array([pts[:,0].mean(), pts[:,1].mean()], dtype=np.float32)

def sort_corners_geometric(centers):
    pts = np.array(centers, dtype=np.float32)
    top = pts[np.argsort(pts[:,1])[:2]]
    bottom = pts[np.argsort(pts[:,1])[2:]]

    return np.array([
        top[np.argmin(top[:,0])],
        top[np.argmax(top[:,0])],
        bottom[np.argmax(bottom[:,0])],
        bottom[np.argmin(bottom[:,0])]
    ], dtype=np.float32)

def draw_keyboard_overlay(frame, H):

    # invert homography to go keyboard → image
    Hinv = np.linalg.inv(H)

    # keyboard corners in mm
    kb_corners = np.array([
        [0, 0],
        [KEYBOARD_WIDTH_MM, 0],
        [KEYBOARD_WIDTH_MM, KEYBOARD_DEPTH_MM],
        [0, KEYBOARD_DEPTH_MM]
    ], dtype=np.float32)

    kb_corners = kb_corners.reshape(-1,1,2)

    # project into image
    img_pts = cv2.perspectiveTransform(kb_corners, Hinv)

    img_pts = np.int32(img_pts)

    # draw keyboard outline
    cv2.polylines(frame, [img_pts], True, (255,0,0), 3)
        # draw key centers
    for label, key in KEY_MAP.items():

        pt = np.array([[key["cx_mm"], key["cy_mm"]]], dtype=np.float32)
        pt = pt.reshape(-1,1,2)

        img_pt = cv2.perspectiveTransform(pt, Hinv)
        x, y = img_pt[0][0]

        cv2.circle(frame, (int(x), int(y)), 4, (0,0,255), -1)

        cv2.putText(
            frame,
            label,
            (int(x)+3, int(y)-3),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.3,
            (0,0,255),
            1
        )


class KeyboardArucoNode(Node):

    def __init__(self):
        super().__init__("keyboard_aruco")

        self.homography_pub = self.create_publisher(Float64MultiArray, "keyboard_plane_homography", 10)
        self.key_positions_pub = self.create_publisher(Float64MultiArray, "keyboard_key_positions", 10)

        # OpenCV camera
        self.cap = cv2.VideoCapture(0)

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        if not self.cap.isOpened():
            self.get_logger().error("Could not open camera")

        self._latest_frame = None
        self._lock = threading.Lock()
        self._running = True

        threading.Thread(target=self._capture_loop, daemon=True).start()

        self.create_timer(1.0/30.0, self._on_timer)

        self.get_logger().info("keyboard_aruco: OpenCV camera 1280x720")

    def _capture_loop(self):

        while self._running:

            ret, frame = self.cap.read()

            if not ret:
                continue

            with self._lock:
                self._latest_frame = frame

    def _on_timer(self):

        with self._lock:
            frame = self._latest_frame

        if frame is None:
            self.homography_pub.publish(Float64MultiArray(data=[0.0]*9))
            return

        corners, ids, _ = cv2.aruco.detectMarkers(frame, ARUCO_DICT, parameters=ARUCO_PARAMS)

        # Draw markers if any are detected
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            status = f"Markers detected: {len(ids)}"
        else:
            status = "No markers detected"

        # Always show camera feed
        cv2.putText(
            frame,
            status,
            (20,40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0,255,0),
            2
        )

        cv2.imshow("Keyboard Camera", frame)
        cv2.waitKey(1)

        # If fewer than 4 markers → can't compute keyboard plane
        if ids is None or len(ids) < 4:
            self.homography_pub.publish(Float64MultiArray(data=[0.0]*9))
            return

        h, w = frame.shape[:2]

        cam_cx = w / 2
        cam_cy = h / 2

        centers = [marker_center(corners[i]) for i in range(4)]
        center = np.mean(centers, axis=0)

        cx = float(center[0] - cam_cx)
        cy = float(cam_cy - center[1])   # flip Y so up is positive

        cv2.circle(frame, (int(cx), int(cy)), 8, (0,255,255), -1)
        cv2.putText(
            frame,
            "Keyboard Center",
            (int(cx)+10, int(cy)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0,255,255),
            2
        )

        src = sort_corners_geometric(centers)

        H, _ = cv2.findHomography(src, DST_PTS)

        if H is None:
            self.homography_pub.publish(Float64MultiArray(data=[0.0]*9))
            return
        if H is not None:
            draw_keyboard_overlay(frame, H)
        self.homography_pub.publish(Float64MultiArray(data=H.flatten().tolist()))
        center_msg = Float64MultiArray()
        center_msg.data = [cx, cy]

        self.key_positions_pub.publish(String(data=json.dumps(center_msg)))

        self.get_logger().info("keyboard locked", throttle_duration_sec=1.0)
   

def main(args=None):

    rclpy.init(args=args)

    node = KeyboardArucoNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()