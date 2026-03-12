# Keyboard ArUco node. Uses DepthAI cam + 4 markers on K552 to publish homography and key positions.
# Topics only (no actions/services). arm just subscribes and uses the data when it needs to type.

import json
import threading
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

import depthai as dai

# K552 physical size (mm). Homography maps image -> this plane.
KEYBOARD_WIDTH_MM = 354.0
KEYBOARD_DEPTH_MM = 123.0
PITCH = 19.05  # 1u key spacing
H_KEY = PITCH
ROW_Y = {'fn': 6.5, 'num': 25.55, 'tab': 44.60, 'caps': 63.65, 'shft': 82.70, 'spc': 101.75}
ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
ARUCO_PARAMS = cv2.aruco.DetectorParameters()
# Keyboard corners in mm: TL, TR, BR, BL (must match order from sort_corners_geometric)
DST_PTS = np.array([
    [0.0, 0.0], [KEYBOARD_WIDTH_MM, 0.0],
    [KEYBOARD_WIDTH_MM, KEYBOARD_DEPTH_MM], [0.0, KEYBOARD_DEPTH_MM],
], dtype=np.float32)

# Key layout: (label, cx_mm, cy_mm, w_mm) per key. Matches keyboard_aruco_laptop_test.
KEY_LAYOUT = []


def u(n):
    """Units to mm (1u = PITCH)."""
    return n * PITCH


def add_row(keys, y, start_x):
    """Append a row of keys to KEY_LAYOUT. keys = [(label, width_units), ...]."""
    x = start_x
    for label, w in keys:
        w_mm = u(w)
        KEY_LAYOUT.append((label, x + w_mm / 2, y, w_mm))
        x += w_mm
# function row
KEY_LAYOUT.append(('Esc', u(0.0) + 4.5 + u(0.5), ROW_Y['fn'], u(1.0)))
fx = u(2.1) + 4.5
for i, name in enumerate(['F1', 'F2', 'F3', 'F4']):
    KEY_LAYOUT.append((name, fx + i * u(1.0) + u(0.5), ROW_Y['fn'], u(1.0)))
fx2 = fx + 4 * u(1.0) + u(0.25)
for i, name in enumerate(['F5', 'F6', 'F7', 'F8']):
    KEY_LAYOUT.append((name, fx2 + i * u(1.0) + u(0.5), ROW_Y['fn'], u(1.0)))
fx3 = fx2 + 4 * u(1.0) + u(0.25)
for i, name in enumerate(['F9', 'F10', 'F11', 'F12']):
    KEY_LAYOUT.append((name, fx3 + i * u(1.0) + u(0.5), ROW_Y['fn'], u(1.0)))
fx4 = fx3 + 4 * u(1.0) + u(0.5)
for i, name in enumerate(['PrtSc', 'ScrLk', 'Pause']):
    KEY_LAYOUT.append((name, fx4 + i * u(1.0) + u(0.5), ROW_Y['fn'], u(1.0)))
# number row
add_row([('`',1),('1',1),('2',1),('3',1),('4',1),('5',1),('6',1),('7',1),('8',1),('9',1),('0',1),('-',1),('=',1),('Bksp',2)], ROW_Y['num'], 4.5)
for i, lbl in enumerate(['Ins','Home','PgUp']): KEY_LAYOUT.append((lbl, u(14.75)+4.5+i*u(1)+u(0.5), ROW_Y['num'], u(1)))
# QWERTY row
add_row([('Tab',1.5),('Q',1),('W',1),('E',1),('R',1),('T',1),('Y',1),('U',1),('I',1),('O',1),('P',1),('[',1),(']',1),('\\',1.5)], ROW_Y['tab'], 4.5)
for i, lbl in enumerate(['Del','End','PgDn']): KEY_LAYOUT.append((lbl, u(14.75)+4.5+i*u(1)+u(0.5), ROW_Y['tab'], u(1)))
# home row
add_row([('Caps',1.75),('A',1),('S',1),('D',1),('F',1),('G',1),('H',1),('J',1),('K',1),('L',1),(';',1),("'",1),('Enter',2.25)], ROW_Y['caps'], 4.5)
# shift row
add_row([('LShft',2.25),('Z',1),('X',1),('C',1),('V',1),('B',1),('N',1),('M',1),(',',1),('.',1),('/',1),('RShft',2.75)], ROW_Y['shft'], 4.5)
KEY_LAYOUT.append(('Up', u(14.75)+4.5+u(1.5), ROW_Y['shft'], u(1)))
# bottom row
add_row([('LCtrl',1.25),('LWin',1.25),('LAlt',1.25),('Space',6.25),('RAlt',1.25),('Fn',1.25),('RCtrl',1.25)], ROW_Y['spc'], 4.5)
KEY_LAYOUT.append(('Left', u(14.75)+4.5+u(0.5), ROW_Y['spc'], u(1)))
KEY_LAYOUT.append(('Down', u(14.75)+4.5+u(1.5), ROW_Y['spc'], u(1)))
KEY_LAYOUT.append(('Right', u(14.75)+4.5+u(2.5), ROW_Y['spc'], u(1)))
KEY_MAP = {lbl: {'cx_mm': cx, 'cy_mm': cy, 'w_mm': w, 'h_mm': H_KEY} for lbl, cx, cy, w in KEY_LAYOUT}
KB_CENTER_X = KEYBOARD_WIDTH_MM / 2.0
KB_CENTER_Y = KEYBOARD_DEPTH_MM / 2.0
Z_MM = 10.0  # height above keyboard plane for key press (mm)


def char_to_key_label(ch):
    """Map typed char to KEY_MAP label. Returns None if not found."""
    upper = ch.upper()
    if upper in KEY_MAP:
        return upper
    special = {" ": "Space", "\n": "Enter", "\t": "Tab"}
    return special.get(ch)


def marker_center(corners_one):
    """Center of one ArUco marker (4 corners -> single point)."""
    pts = corners_one.reshape((4, 2))
    return np.array([pts[:, 0].mean(), pts[:, 1].mean()], dtype=np.float32)


def inside_corner(corners_one, all_centers):
    """Vertex of marker closest to centroid of all centers (keyboard center)."""
    pts = corners_one.reshape((4, 2))
    center = np.mean(all_centers, axis=0)
    dists = np.linalg.norm(pts - center, axis=1)
    return np.array(pts[np.argmin(dists)], dtype=np.float32)


def sort_corners_geometric(centers):
    """Order 4 marker centers as TL, TR, BR, BL for homography."""
    pts = np.array(centers, dtype=np.float32)
    top = pts[np.argsort(pts[:, 1])[:2]]
    bottom = pts[np.argsort(pts[:, 1])[2:]]
    return np.array([
        top[np.argmin(top[:, 0])], top[np.argmax(top[:, 0])],
        bottom[np.argmax(bottom[:, 0])], bottom[np.argmin(bottom[:, 0])]
    ], dtype=np.float32)


class KeyboardArucoNode(Node):
    """Publishes keyboard homography and key positions; types sequence from stdin, sends one movement per arm request."""

    def __init__(self):
        super().__init__("keyboard_aruco")
        # Publishers: homography (9 floats), key map (JSON), movement vector (JSON)
        self.homography_pub = self.create_publisher(Float64MultiArray, "keyboard_plane_homography", 10)
        self.key_positions_pub = self.create_publisher(String, "keyboard_key_positions", 10)
        self.movement_pub = self.create_publisher(String, "keyboard_movement_vector", 10)
        self.create_subscription(String, "keyboard_key_request", self._on_key_request, 10)
        self._launchpad_keys = []
        self._launchpad_idx = 0
        self._launchpad_lock = threading.Lock()

        # DepthAI camera, 1280x720
        pipeline = dai.Pipeline()
        cam = pipeline.create(dai.node.Camera).build()
        self._video_queue = cam.requestOutput((1280, 720)).createOutputQueue()
        pipeline.start()
        self._pipeline = pipeline

        self._latest_frame = None
        self._lock = threading.Lock()
        self._running = True
        self._last_log_time = 0.0
        threading.Thread(target=self._capture_loop, daemon=True).start()
        threading.Thread(target=self._input_loop, daemon=True).start()
        self.create_timer(1.0 / 30.0, self._on_timer)  # 30 Hz detect + publish
        self.get_logger().info(f"keyboard_aruco: DepthAI 1280x720, {len(KEY_MAP)} keys. Type a sequence and press Enter, then arm requests trigger each key.")

    def _input_loop(self):
        """Background thread: prompt for sequence string, update _launchpad_keys on Enter."""
        while self._running:
            try:
                line = input("Sequence: ").strip()
                if not line:
                    continue
                keys = []
                for ch in line:
                    label = char_to_key_label(ch)
                    if label is not None:
                        keys.append(label)
                with self._launchpad_lock:
                    self._launchpad_keys = keys
                    self._launchpad_idx = 0
                self.get_logger().info(f"Sequence set: {len(keys)} keys")
            except (EOFError, KeyboardInterrupt):
                break

    def _on_key_request(self, msg):
        """On arm 'ready for next key': publish movement (dx_mm, dy_mm, z_mm) from keyboard center, advance index."""
        with self._launchpad_lock:
            if self._launchpad_idx >= len(self._launchpad_keys):
                return
            key = self._launchpad_keys[self._launchpad_idx]
            self._launchpad_idx += 1
        info = KEY_MAP[key]
        dx_mm = info["cx_mm"] - KB_CENTER_X
        dy_mm = info["cy_mm"] - KB_CENTER_Y
        payload = {"key": key, "dx_mm": dx_mm, "dy_mm": dy_mm, "z_mm": Z_MM}
        self.movement_pub.publish(String(data=json.dumps(payload)))

    def _capture_loop(self):
        """Background thread: pull frames from DepthAI into _latest_frame."""
        while self._running:
            try:
                frame = self._video_queue.get().getCvFrame()
                with self._lock:
                    self._latest_frame = frame
            except Exception:
                break

    def _on_timer(self):
        """30 Hz: detect 4 ArUco markers, compute homography, publish homography + key_positions or invalid sentinel."""
        with self._lock:
            frame = self._latest_frame
        if frame is None:
            self.homography_pub.publish(Float64MultiArray(data=[0.0] * 9))  # invalid sentinel
            return
        corners, ids, _ = cv2.aruco.detectMarkers(frame, ARUCO_DICT, parameters=ARUCO_PARAMS)
        if ids is None or len(corners) < 4:
            self.homography_pub.publish(Float64MultiArray(data=[0.0] * 9))
            return
        # Pick 4 geometric corners (TL, TR, BR, BL) from all detected markers
        centers = [marker_center(c) for c in corners]
        pts = np.array(centers, dtype=np.float32)
        sorted_by_y = pts[np.argsort(pts[:, 1])]
        top2 = sorted_by_y[:2]
        bottom2 = sorted_by_y[-2:]
        tl = top2[np.argmin(top2[:, 0])]
        tr = top2[np.argmax(top2[:, 0])]
        bl = bottom2[np.argmin(bottom2[:, 0])]
        br = bottom2[np.argmax(bottom2[:, 0])]
        corner_centers = np.array([tl, tr, br, bl], dtype=np.float32)
        corner_indices = [
            np.argmin([np.linalg.norm(c - p) for c in centers])
            for p in corner_centers
        ]
        if len(set(corner_indices)) != 4:
            self.homography_pub.publish(Float64MultiArray(data=[0.0] * 9))
            return
        # Homography from image (marker inside-corners) to keyboard mm
        src = np.array([
            inside_corner(corners[corner_indices[k]], corner_centers)
            for k in range(4)
        ], dtype=np.float32)
        H, _ = cv2.findHomography(src, DST_PTS)
        if H is None:
            self.homography_pub.publish(Float64MultiArray(data=[0.0] * 9))
            return
        self.homography_pub.publish(Float64MultiArray(data=H.flatten().tolist()))  # 3x3 row-major
        self.key_positions_pub.publish(String(data=json.dumps(KEY_MAP)))
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._last_log_time >= 1.0:
            self._last_log_time = now
            self.get_logger().info("keyboard locked")


def main(args=None):
    """Run keyboard_aruco node (DepthAI + ArUco, homography, sequence from stdin, movement on request)."""
    rclpy.init(args=args)
    node = KeyboardArucoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
