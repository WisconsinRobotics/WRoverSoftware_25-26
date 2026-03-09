# Keyboard ArUco node. Uses DepthAI cam + 4 markers on K552 to publish homography and key positions.
# Topics only (no actions/services). Arm just subscribes and uses the data when it needs to type.
# Run: python3 keyboard_aruco_node_fixed.py  |  Test: ros2 topic echo /keyboard_plane_homography

import json
import threading
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

import depthai as dai

# K552 physical size and ArUco target corners (mm). DST_PTS = BL, BR, TR, TL.
KEYBOARD_WIDTH_MM = 354.0
KEYBOARD_DEPTH_MM = 123.0
PITCH = 19.05
H_KEY = PITCH
ROW_Y = {'fn': 6.5, 'num': 25.55, 'tab': 44.60, 'caps': 63.65, 'shft': 82.70, 'spc': 101.75}
ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
ARUCO_PARAMS = cv2.aruco.DetectorParameters()
DST_PTS = np.array([
    [0.0, 0.0], [KEYBOARD_WIDTH_MM, 0.0],
    [KEYBOARD_WIDTH_MM, KEYBOARD_DEPTH_MM], [0.0, KEYBOARD_DEPTH_MM],
], dtype=np.float32)

# Key layout: (label, cx_mm, cy_mm, w_mm) per key. u() = units to mm.
KEY_LAYOUT = []
def u(n): return n * PITCH
def add_row(keys, y, start_x):
    x = start_x
    for label, w in keys:
        w_mm = u(w)
        KEY_LAYOUT.append((label, x + w_mm / 2, y, w_mm))
        x += w_mm
add_row([('`',1),('1',1),('2',1),('3',1),('4',1),('5',1),('6',1),('7',1),('8',1),('9',1),('0',1),('-',1),('=',1),('Bksp',2)], ROW_Y['num'], 4.5)
for i, lbl in enumerate(['Ins','Home','PgUp']): KEY_LAYOUT.append((lbl, u(14.75)+4.5+i*u(1)+u(0.5), ROW_Y['num'], u(1)))
add_row([('Tab',1.5),('Q',1),('W',1),('E',1),('R',1),('T',1),('Y',1),('U',1),('I',1),('O',1),('P',1),('[',1),(']',1),('\\',1.5)], ROW_Y['tab'], 4.5)
for i, lbl in enumerate(['Del','End','PgDn']): KEY_LAYOUT.append((lbl, u(14.75)+4.5+i*u(1)+u(0.5), ROW_Y['tab'], u(1)))
add_row([('Caps',1.75),('A',1),('S',1),('D',1),('F',1),('G',1),('H',1),('J',1),('K',1),('L',1),(';',1),("'",1),('Enter',2.25)], ROW_Y['caps'], 4.5)
add_row([('LShft',2.25),('Z',1),('X',1),('C',1),('V',1),('B',1),('N',1),('M',1),(',',1),('.',1),('/',1),('RShft',2.75)], ROW_Y['shft'], 4.5)
KEY_LAYOUT.append(('Up', u(14.75)+4.5+u(1.5), ROW_Y['shft'], u(1)))
add_row([('LCtrl',1.25),('LWin',1.25),('LAlt',1.25),('Space',6.25),('RAlt',1.25),('Fn',1.25),('RCtrl',1.25)], ROW_Y['spc'], 4.5)
KEY_LAYOUT.append(('Left', u(14.75)+4.5+u(0.5), ROW_Y['spc'], u(1)))
KEY_LAYOUT.append(('Down', u(14.75)+4.5+u(1.5), ROW_Y['spc'], u(1)))
KEY_LAYOUT.append(('Right', u(14.75)+4.5+u(2.5), ROW_Y['spc'], u(1)))
KEY_MAP = {lbl: {'cx_mm': cx, 'cy_mm': cy, 'w_mm': w, 'h_mm': H_KEY} for lbl, cx, cy, w in KEY_LAYOUT}


def marker_center(corners_one):
    pts = corners_one.reshape((4, 2))
    return np.array([pts[:, 0].mean(), pts[:, 1].mean()], dtype=np.float32)


def sort_corners_geometric(centers):
    #order 4 corners as Tl TR BL BR
    pts = np.array(centers, dtype=np.float32)
    top = pts[np.argsort(pts[:, 1])[:2]]
    bottom = pts[np.argsort(pts[:, 1])[2:]]
    return np.array([
        top[np.argmin(top[:, 0])], top[np.argmax(top[:, 0])],
        bottom[np.argmax(bottom[:, 0])], bottom[np.argmin(bottom[:, 0])]
    ], dtype=np.float32)


class KeyboardArucoNode(Node):
    def __init__(self):
        super().__init__("keyboard_aruco")
        # NO ACTIONS HERE RIGHT NOW, JUST SENDS AND Waits for result from arms state machine
        self.homography_pub = self.create_publisher(Float64MultiArray, "keyboard_plane_homography", 10)
        self.key_positions_pub = self.create_publisher(String, "keyboard_key_positions", 10)

        # timer to run arucos and publishes/setup for DAI
        pipeline = dai.Pipeline()
        cam = pipeline.create(dai.node.Camera).build()
        self._video_queue = cam.requestOutput((1280, 720)).createOutputQueue()
        pipeline.start()
        self._pipeline = pipeline  # keep ref

        self._latest_frame = None
        self._lock = threading.Lock()
        self._running = True
        threading.Thread(target=self._capture_loop, daemon=True).start()
        self.create_timer(1.0 / 30.0, self._on_timer)
        self.get_logger().info(f"keyboard_aruco: DepthAI 1280x720, {len(KEY_MAP)} keys")

    def _capture_loop(self):
        # make sure to get latest frame
        while self._running:
            try:
                frame = self._video_queue.get().getCvFrame()
                with self._lock:
                    self._latest_frame = frame
            except Exception:
                break

    def _on_timer(self):
        with self._lock:
            frame = self._latest_frame
        if frame is None:
            self.homography_pub.publish(Float64MultiArray(data=[0.0] * 9))
            return
        corners, ids, _ = cv2.aruco.detectMarkers(frame, ARUCO_DICT, parameters=ARUCO_PARAMS)
        # Need 4 markers to define the plane; otherwise send zeros so arm knows not to use it
        if ids is None or len(ids) < 4:
            self.homography_pub.publish(Float64MultiArray(data=[0.0] * 9))
            return
        centers = [marker_center(corners[i]) for i in range(4)]
        src = sort_corners_geometric(centers)
        H, _ = cv2.findHomography(src, DST_PTS)
        if H is None:
            self.homography_pub.publish(Float64MultiArray(data=[0.0] * 9))
            return
        self.homography_pub.publish(Float64MultiArray(data=H.flatten().tolist()))
        self.key_positions_pub.publish(String(data=json.dumps(KEY_MAP)))
        self.get_logger().info("keyboard locked", throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardArucoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
