import cv2
import zmq
import base64
import numpy as np
import socket
import argparse
import threading
import signal
from typing import List
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray


import json
import threading
import argparse


import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
HAS_ROS = True


# ── keyboard physical layout (mm) ──────────────────────────────────────────────

KEYBOARD_W = 354.0
KEYBOARD_D = 123.0
PITCH = 19.05  # 1u key spacing, standard across most mechs

# where the arm rests in keyboard mm space (0,0 = top-left of board)
KB_CX = 174.8
KB_CY = 56.5
Z_MM = 10.0  # press depth, needs tuning on real board

# nudge every arm command if it consistently misses one direction
# only affects what gets sent to the arm, overlay stays unchanged
OFFSET_X = 0.0  # + = right
OFFSET_Y = 0.0  # + = up (same convention as MOVES dy)

# vectors from arm home to each key (dx+ right, dy+ up in arm frame)
MOVES = {
    'A': (-127.14, -10.18),
    'B': (-41.41, -29.23),
    'C': (-79.51, -29.23),
    'D': (-89.04, -10.18),
    'E': (-93.8, 8.88),
    'F': (-69.99, -10.18),
    'G': (-50.94, -10.18),
    'H': (-31.89, -10.18),
    'I': (1.45, 8.88),
    'J': (-12.84, -10.18),
    'K': (6.21, -10.18),
    'L': (25.26, -10.18),
    'M': (-3.31, -29.23),
    'N': (-22.36, -29.23),
    'O': (20.5, 8.88),
    'P': (39.55, 8.88),
    'Q': (-131.9, 8.88),
    'R': (-74.75, 8.88),
    'S': (-108.09, -10.18),
    'T': (-55.7, 8.88),
    'U': (-17.6, 8.88),
    'V': (-60.46, -29.23),
    'W': (-112.85, 8.88),
    'X': (-98.56, -29.23),
    'Y': (-36.65, 8.88),
    'Z': (-117.61, -29.23),
    '0': (0.0, 0.0),
    '1': (0.0, 0.0),
    '2': (0.0, 0.0),
    '3': (0.0, 0.0),
    '4': (0.0, 0.0),
    '5': (0.0, 0.0),
    '6': (0.0, 0.0),
    '7': (0.0, 0.0),
    '8': (0.0, 0.0),
    '9': (0.0, 0.0),
    'Space': (-39.03, -48.28),
    'Enter': (94.32, -10.18),
}

# wider-than-1u keys for the overlay
WIDTHS = {
    'Space': PITCH * 6.25,
    'Enter': PITCH * 2.25,
    'Bksp':  PITCH * 2.0,
    'Tab':   PITCH * 1.5,
}

# build absolute key positions from MOVES for the overlay
# MOVES dy+ = up, but pixel y+ = down, so we negate dy
KEYS = {}
for label, (dx, dy) in MOVES.items():
    KEYS[label] = {
        'cx_mm': KB_CX + dx,
        'cy_mm': KB_CY - dy,
        'w_mm': WIDTHS.get(label, PITCH),
        'h_mm': PITCH,
    }


# ── aruco config ───────────────────────────────────────────────────────────────


MARKER_ID_TO_CORNER = {
    1: 0,   # marker ID 1 at TL corner
    4: 1,   #4 at TR
    3: 2,   #3 at BR
    2: 3,   #2 at BL
}
_PAIR_QUALITY = {
    frozenset({0, 2}): 'diagonal',   
    frozenset({1, 3}): 'diagonal',   
    frozenset({0, 1}): 'same_edge',  
    frozenset({2, 3}): 'same_edge',  
    frozenset({0, 3}): 'same_edge',
    frozenset({1, 2}): 'same_edge',  
}

ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
ARUCO_PARAMS = cv2.aruco.DetectorParameters()
ARUCO_PARAMS.adaptiveThreshWinSizeMin = 5
ARUCO_PARAMS.adaptiveThreshWinSizeMax = 21
ARUCO_PARAMS.adaptiveThreshWinSizeStep = 4
ARUCO_PARAMS.adaptiveThreshConstant = 7

# the 4 inside corners of our markers should map to these keyboard corners
KB_CORNERS_MM = np.array([
    [0.0, 0.0],             # TL
    [KEYBOARD_W, 0.0],      # TR
    [KEYBOARD_W, KEYBOARD_D],  # BR
    [0.0, KEYBOARD_D],      # BL
], dtype=np.float32)

# CLAHE helps even out glare/reflections before marker detection
CLAHE = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))


def start_multiple_receivers(ip: str, ports: List[int]):
    stop_event = threading.Event()
    threads: List[threading.Thread] = []
    frames: dict = {}
    lock = threading.Lock()
    stats = {}

    for port in ports:
        t = threading.Thread(
            target=receive_camera_data, 
            args=(ip, port, stop_event, frames, lock, stats), 
            daemon=True
        )
        t.start()
        threads.append(t)
        print(f"Started receiver for {ip}:{port}")

    return stop_event, threads, frames, lock, stats

 #── aruco helpers ──────────────────────────────────────────────────────────────

def preprocess_for_aruco(frame):
    """grayscale + CLAHE so specular glare doesn't wash out markers"""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return CLAHE.apply(gray)


def marker_center(one_corner):
    """average of a marker's 4 vertices = its center"""
    return one_corner.reshape(4, 2).mean(axis=0).astype(np.float32)


def inside_corner(one_corner, all_centers):
    """the vertex closest to the centroid of all markers is the inside corner,
    i.e. the one pointing toward the middle of the keyboard"""
    pts = one_corner.reshape(4, 2)
    centroid = np.mean(all_centers, axis=0)
    dists = np.linalg.norm(pts - centroid, axis=1)
    return pts[np.argmin(dists)].astype(np.float32)


def sort_four_corners(points):
    """sort 4 points into TL, TR, BR, BL order by position"""
    pts = np.array(points, dtype=np.float32)
    by_y = np.argsort(pts[:, 1])
    top, bot = by_y[:2], by_y[2:]
    tl = top[np.argmin(pts[top, 0])]
    tr = top[np.argmax(pts[top, 0])]
    bl = bot[np.argmin(pts[bot, 0])]
    br = bot[np.argmax(pts[bot, 0])]
    return [tl, tr, br, bl]

#Check if the two detected are diagonal for similarity transform 
def check_two_marker_quality(corners, ids):
    if len(corners) != 2 or ids is None or len(ids) != 2:
        return False
    #get id 
    ca = MARKER_ID_TO_CORNER.get(int(ids[0][0]))
    cb = MARKER_ID_TO_CORNER.get(int(ids[1][0]))
    
    if ca is None or cb is None:
        return False
    pair = frozenset({ca, cb})
    quality = _PAIR_QUALITY.get(pair)
    return quality

def compute_homography(corners, ids):
    if ids is None or len(corners) < 2:
        return None, None
    
    # Build matched pairs w id to corner
    flat_ids = ids.flatten()
    all_centers = [marker_center(c) for c in corners]
    src_pts, dst_pts = [], []
    for i, mid in enumerate(flat_ids):
        corner_idx = MARKER_ID_TO_CORNER.get(int(mid))
        if corner_idx is None:
            continue   # unknown marker ID, skip
        src_pts.append(inside_corner(corners[i], all_centers))
        dst_pts.append(KB_CORNERS_MM[corner_idx])
        
    n = len(src_pts)
    if n < 2:
        return None, None

    src = np.array(src_pts, dtype=np.float32) #for transforms we need float32
    dst = np.array(dst_pts, dtype=np.float32)
    
    # Homography possible 
    if n >= 4:
        H, _ = cv2.findHomography(src, dst)
        if H is None:
            return None, None
        return H, src
    #affine transform
    if n == 3:
        M = cv2.getAffineTransform(src[:3], dst[:3])
        H = np.vstack([M, [0.0, 0.0, 1.0]])
        return H, src

    # Similarity (4 DOF): rotation + uniform scale + translation
    if check_two_marker_quality(corners, ids) == 'diagonal':
        M, inliers = cv2.estimateAffinePartial2D(src, dst)
        if M is None:
            return None, None
        H = np.vstack([M, [0.0, 0.0, 1.0]])
        return H, src
    else:
        return None, None


def mm_to_px(H_inv, x_mm, y_mm):
    """keyboard mm coords -> pixel coords using inverse homography"""
    pt = np.array([[[x_mm, y_mm]]], dtype=np.float32)
    out = cv2.perspectiveTransform(pt, H_inv)
    return int(out[0, 0, 0]), int(out[0, 0, 1])


def char_to_label(ch):
    """turn a typed character into a MOVES key label"""
    up = ch.upper()
    if up in MOVES:
        return up
    return {' ': 'Space', '\n': 'Enter', '\t': 'Tab'}.get(ch)
def receive_camera_data(ip: str, port: int, stop_event: threading.Event, frames: dict, lock: threading.Lock, stats: dict):
    """Subscribe to a single publisher at ip:port and display frames until stop_event is set."""
    context = zmq.Context()
    footage_socket = context.socket(zmq.SUB)
    footage_socket.setsockopt(zmq.CONFLATE, 1)
    footage_socket.connect(f'tcp://{ip}:{port}')
    footage_socket.setsockopt_string(zmq.SUBSCRIBE, '')
    # set a receive timeout so we can check stop_event periodically
    footage_socket.setsockopt(zmq.RCVTIMEO, 500)

    window_name = f"Stream-{port}"
    print(f"Receiving data on {ip}:{port} -> window '{window_name}'")
    print("Press 'q' in any window to quit")

    try:
        while not stop_event.is_set():
            try:
                frame_bytes = footage_socket.recv()
            except zmq.Again:
                continue

            try:
                with lock:
                    stats[window_name] = stats.get(window_name, 0) + len(frame_bytes)
            except Exception:
                pass

            try:
                img = base64.b64decode(frame_bytes)
                npimg = np.frombuffer(img, dtype=np.uint8)
                source = cv2.imdecode(npimg, cv2.IMREAD_COLOR)
                if source is None:
                    continue
                # store the latest frame for the main thread to display
                try:
                    with lock:
                        frames[window_name] = source.copy()
                except Exception:
                    # if storing fails, skip this frame
                    continue
            except Exception:
                # skip malformed frames
                continue
    finally:
        # remove any stored frame for this window
        try:
            with lock:
                if window_name in frames:
                    del frames[window_name]
                if window_name in stats:
                    del stats[window_name]
        except Exception:
            pass
        try:
            footage_socket.close()
        except Exception:
            pass
        try:
            context.term()
        except Exception:
            pass


class CameraReceiver(Node):
    def __init__(self):
        super().__init__('camera_receiver')
        self.keyboard_center= self.create_publisher(Float64MultiArray, 'keyboard_center_2', 10)

        self.key_center = Float64MultiArray()
        self.key_center.data = [0.0, 0.0]

        parser = argparse.ArgumentParser(prog='camera_receiver', description='Receive one or more camera streams over ZMQ')
        parser.add_argument('--broadcast-ip', type=str, default="0.0.0.0", help='IP of the publisher(s)')
        parser.add_argument('--base-port', type=int, default=5555, help='Starting port for the first camera. Subsequent cameras use base-port+index')
        parser.add_argument('--count', type=int, default=1, help='Number of sequential ports to subscribe to starting at base-port')
        parser.add_argument('--show-stats', type=str, choices=['on', 'off'], default='off', help='Show streaming statistics')

        args = parser.parse_args()
        broadcast_ip = "192.168.1.192"
        port = "5555"
        self.show_stats = args.show_stats == 'on'

        #self.stop_event, self.threads, self.frames, self.lock, self.stats = start_multiple_receivers(broadcast_ip, ports)

        self.stop_event = threading.Event()
        self.frames: dict = {}
        self.lock = threading.Lock()
        self.stats = {}

        
        t = threading.Thread(
            target=receive_camera_data, 
            args=(broadcast_ip, port, self.stop_event, self.frames, self.lock, self.stats), 
            daemon=True
        )
        t.start()
     
        print(f"Started receiver for {broadcast_ip}:{port}")

        self.get_logger().info(f"AAAAAAA: {self.frames}")
        def _signal_handler(signum, frame):
            print('Stopping receivers...')
            self.stop_event.set()
            rclpy.shutdown()

        signal.signal(signal.SIGINT, _signal_handler)
        signal.signal(signal.SIGTERM, _signal_handler)

        self.prev_bytes = {}
        prev_time = time.time()
        last_data_rate_update_time = time.time()
        data_rate_text = "0 KB/s"
        self.frame = None

        self.timer = self.create_timer(0.1, self._timer_callback)
        


    def _timer_callback(self):
        """periodic timer to resend current key if we're waiting for arm response"""

        
        #receive camera data
        with self.lock:
            keys = list(self.frames.keys())
        for k in keys:
            
            with self.lock:
                
                self.frame = self.frames.get(k)
            if self.frame is None:
                continue
            
            if self.show_stats:
                now = time.time()
                if now - last_data_rate_update_time >= 1.0:
                    with self.lock:
                        total = self.stats.get(k, 0)
                    prev = self.prev_bytes.get(k, 0)
                    rate_bps = (total - prev) / max(1e-6, now - last_data_rate_update_time)
                    self.prev_bytes[k] = total

                    data_rate_text = f"{rate_bps/1024:.1f} KB/s"
                    last_data_rate_update_time = now
                cv2.putText(self.frame, data_rate_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow(k, self.frame)
       
        #self.get_logger().info(f"self.frame: {self.frame}")
        if self.frame is None:
            return
        frame = self.frame.copy()
    

        # detect markers (CLAHE helps with glare on markers)
        enhanced = preprocess_for_aruco(frame)
        corners, ids, _ = cv2.aruco.detectMarkers(enhanced, ARUCO_DICT, parameters=ARUCO_PARAMS)
        self.get_logger().info(f"IDS: {ids}")
        # throw out false positives from keycap icons / LED labels
        if ids is not None:
            MIN_AREA = 500
            MIN_ASPECT = 0.6
            MAX_ASPECT = 1.4
            mask = []
            for i, c in enumerate(corners):
                pts = c.reshape(4, 2)
                area = cv2.contourArea(pts)
                if area < MIN_AREA:
                    continue
                x, y, w, h = cv2.boundingRect(pts)
                if h == 0:
                    continue
                aspect = w / h
                if aspect < MIN_ASPECT or aspect > MAX_ASPECT:
                    continue
                mask.append(i)
            corners = [corners[i] for i in mask]
            ids = ids[mask] if len(mask) > 0 else None

        # try to get a fresh homography
        H, self.src_pts = compute_homography(corners, ids)
        # Example: src_points = np.array([[x1, y1], [x2, y2], [x3, y3], [x4, y4]])
        

        #Getting center of keyboard in pixel coordinates and publishing to ROS topic
        if self.src_pts is not None and len(self.src_pts) > 0:
            center = np.mean(self.src_pts, axis=0)  # shape (2,)
            x_center, y_center = center
            x_center = -640/2 + x_center
            y_center = 480/2 - y_center
            
            
            self.get_logger().info(f"Center: {x_center}, {y_center}")
            self.key_center.data = [float(x_center), float(y_center)]
            self.keyboard_center.publish(self.key_center)






def main(args=None):
    # if not cap.isOpened():
    #     msg = "Couldn't open webcam index 1"
    #     if node:
    #         node.get_logger().error(msg)
    #         node.destroy_node()
    #         rclpy.shutdown()
    #     else:
    #         print(msg)
    #     return
   
    rclpy.init(args=args)

    node = CameraReceiver()
    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


    print("Q=quit  L=launchkey  P=print\n")





if __name__ == "__main__":
    main()