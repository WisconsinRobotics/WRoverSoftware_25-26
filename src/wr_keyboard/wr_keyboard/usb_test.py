from typing import List
from wr_keyboard.wr_keyboard.keyboard_test import KEYS, LOCK_TIMEOUT, MOVES
import zmq
import base64
import socket
import sys
import cv2
import numpy as np
import json
import time
import threading
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from std_msgs.msg import Float64MultiArray
    
#USB physical layout 

USB_W = 40
USB_D = 40
KB_CX = 20
KB_CY = 20
# nudge every arm command if it consistently misses one direction
# only affects what gets sent to the arm, overlay stays unchanged
OFFSET_X = 0.0  # + = right
OFFSET_Y = 0.0  # + = up (same convention as MOVES dy)
# vectors from arm home to each key (dx+ right, dy+ up in arm frame)
# the 4 inside corners of our markers should map to these keyboard corners
USB_CORNERS_MM = np.array([
    [0.0, 0.0],             # TL
    [USB_W, 0.0],      # TR
    [USB_W, USB_D],  # BR
    [0.0, USB_D],      # BL
], dtype=np.float32)

# ── aruco config ───────────────────────────────────────────────────────────────

ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
ARUCO_PARAMS = cv2.aruco.DetectorParameters()
ARUCO_PARAMS.adaptiveThreshWinSizeMin = 3
ARUCO_PARAMS.adaptiveThreshWinSizeMax = 53
ARUCO_PARAMS.adaptiveThreshWinSizeStep = 4
ARUCO_PARAMS.adaptiveThreshConstant = 7
ARUCO_PARAMS.polygonalApproxAccuracyRate = 0.05
ARUCO_PARAMS.minMarkerPerimeterRate = 0.02
ARUCO_PARAMS.errorCorrectionRate = 0.7
ARUCO_PARAMS.perspectiveRemovePixelPerCell = 8
CLAHE = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
#1 - TR, 2- BL, 3- BR, 4- TL
MARKER_ID_TO_CORNER = {
    1: 3,  # bottom-right
    2: 1,  # top-right
    3: 4,  # top-left
    4: 2,   # bottom-left
}
PAIR_QUALITY = {
    frozenset({0, 2}): 'diagonal',   
    frozenset({1, 3}): 'diagonal',   
    frozenset({0, 1}): 'same_edge',  
    frozenset({2, 3}): 'same_edge',  
    frozenset({0, 3}): 'same_edge',
    frozenset({1, 2}): 'same_edge',  
}
# ── aruco helpers ──────────────────────────────────────────────────────────────
def preprocess_for_aruco(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return CLAHE.apply(gray)

def marker_center(one_corner):
    return one_corner.reshape(4, 2).mean(axis=0).astype(np.float32)

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
    quality = PAIR_QUALITY.get(pair)
    return quality

def compute_homography(corners, ids):
    if ids is None or len(corners) < 2:
        return None, None
    
    # Build matched pairs w id to corner
    flat_ids = ids.flatten()
    all_pts = np.vstack([c.reshape(4, 2) for c in corners])
    global_center = np.mean(all_pts, axis=0)
    src_pts, dst_pts = [], []
    for i, mid in enumerate(flat_ids):
        corner_idx = MARKER_ID_TO_CORNER.get(int(mid))
        if corner_idx is None:
            continue   # unknown marker ID, skip
        src_pts.append(inside_corner(corners[i], global_center))
        dst_pts.append(USB_CORNERS_MM[corner_idx])
        
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

def inside_corner(corner, global_center):
    pts = corner.reshape(4, 2)
    dists = np.linalg.norm(pts - global_center, axis=1)
    return pts[np.argmin(dists)]
  
# ── debug overlay drawing ─────────────────────────────────────────────────────
#TODO change to basic id drawing for usb task
def draw_ids(frame, ids, corners, rejected_corners = None, sorted_src = None):
    if ids is None:
        return
    for i in range(len(ids)):
        mid = int(ids[i][0])
        pts = corners[i].reshape(4, 2).astype(np.int32)
        cv2.polylines(frame, [pts], True, (0, 255, 0), 2)
        c = marker_center(corners[i])
        cv2.circle(frame, (int(c[0]), int(c[1])), 5, (255, 0, 0), -1)
        cv2.putText(frame, f"ID {mid}", (pts[0][0], pts[0][1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        if rejected_corners is not None:
            pts_rej = rejected_corners[i].reshape(4, 2).astype(np.int32)
            cv2.polylines(frame, [pts_rej], True, (0, 0, 255), 2)
            rc = marker_center(rejected_corners[i])
            cv2.circle(frame, (int(rc[0]), int(rc[1])), 5, (255, 0, 0), -1)

    if sorted_src is not None:
        for pt, name in zip(sorted_src, ['TL', 'TR', 'BR', 'BL']):
            cv2.circle(frame, (int(pt[0]), int(pt[1])), 6, (0, 0, 255), -1)
            cv2.putText(frame, name, (int(pt[0]) + 8, int(pt[1]) + 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            
#wreciever helpers =======================================================
class FrameStore:
    def __init__(self):
        self._frames: dict = {}
        self._lock = threading.Lock()

    def set_latest(self, stream_name: str, frame: np.ndarray):
        try:
            with self._lock:
                self._frames[stream_name] = frame.copy()
            return None
        except (TypeError, RuntimeError) as exc:
            return exc

    def remove_stream(self, stream_name: str):
        try:
            with self._lock:
                self._frames.pop(stream_name, None)
        except Exception:
            pass

    def snapshot_keys(self) -> List[str]:
        with self._lock:
            return list(self._frames.keys())

    def get_frame(self, stream_name: str):
        with self._lock:
            return self._frames.get(stream_name)
            
#wreciever helpers =======================================================   
def receive_camera_data(
    ip: str,
    port: int,
    timeout: float,
    frame_store: FrameStore,
    stop_event: threading.Event,
):
    """Subscribe to a single publisher at ip:port and display frames until stop_event is set."""
    context = zmq.Context()
    footage_socket = context.socket(zmq.SUB)
    footage_socket.setsockopt(zmq.CONFLATE, 1)
    footage_socket.setsockopt(zmq.LINGER, 0)
    footage_socket.connect(f"tcp://{ip}:{port}")
    footage_socket.setsockopt_string(zmq.SUBSCRIBE, "")
    # set a receive timeout so we can check stop_event periodically
    footage_socket.setsockopt(zmq.RCVTIMEO, 500)
 
    window_name = f"Stream-{port}"
 
    # Verify connection by waiting for first frame (10 second timeout)
    connection_timeout = timeout
    start_time = time.time()
    first_frame_received = False
 
    while not stop_event.is_set() and not first_frame_received:
        try:
            footage_socket.recv(zmq.NOBLOCK)
            first_frame_received = True
        except zmq.Again:
            if time.time() - start_time > connection_timeout:
                return
            time.sleep(0.1)  # ZMQ_CONNECTION_POLL_INTERVAL_SECONDS
            continue
 
    if not first_frame_received:
        return
    try:
        while not stop_event.is_set():
            try:
                frame_bytes = footage_socket.recv()
            except zmq.Again:
                continue
 
            try:
                npimg = np.frombuffer(frame_bytes, dtype=np.uint8)
                source = cv2.imdecode(npimg, cv2.IMREAD_COLOR)
                if source is None:
                    continue
                frame_store.set_latest(window_name, source)
            except ValueError:
                # skip malformed frames
                continue
    finally:
        # remove any stored frame for this window
        footage_socket.close()
        try:
            context.term()
        except Exception:
            pass

def discover_stream_config(
    discovery_port: int, timeout: float, streamer_name_filter: str = None
):
    """Listen for UDP discovery heartbeats and return the first matching stream config."""
    receiver_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    receiver_socket.bind(("", discovery_port))
    receiver_socket.settimeout(0.25)

    print(
        
            f"Listening for discovery on UDP {discovery_port} for up to {timeout:.1f}s"
            f"{f' (filter={streamer_name_filter})' if streamer_name_filter else ''}..."
    )

    deadline = time.time() + max(0.1, timeout)
    try:
        while time.time() < deadline:
            try:
                data, _addr = receiver_socket.recvfrom(4096)
            except socket.timeout:
                continue
            #TODO might break cause no self make print if so
            discovered = parse_discovery_payload(data, streamer_name_filter)
            if discovered is None:
                continue

           
            return discovered
    finally:
        receiver_socket.close()

    return None

def parse_discovery_payload(
    packet: bytes,
    streamer_name_filter = None,
):
    """Parse and validate a discovery packet.

    Returns a normalized dict for valid packets, otherwise None.
    """
    try:
        payload = json.loads(packet.decode("utf8"))
    except (UnicodeDecodeError, json.JSONDecodeError):
        return None

    if payload.get("type") != "WRECORDER_DISCOVERY":
        return None
    if payload.get("version") != 1:
        return None

    streamer_name = str(payload.get("streamer_name", "")).strip()
    streamer_ip = str(payload.get("streamer_ip", "")).strip()
    base_port = payload.get("base_port")
    stream_count = payload.get("stream_count")

    if streamer_name_filter and streamer_name != streamer_name_filter:
        return None

    if not streamer_ip or not streamer_name:
        return None

    if not (0 <= base_port <= 65535):
        return None

    if not isinstance(stream_count, int) or stream_count < 1:
        return None

    return {
        "streamer_name": streamer_name,
        "streamer_ip": streamer_ip,
        "base_port": base_port,
        "stream_count": stream_count,
    }
            
class UsbNode(Node):

    def __init__(self):
        super().__init__('usb_aruco')
        #usb args
        self.usb_center_pub = self.create_publisher(Float64MultiArray, 'usb_center', 10)
        self._lock = threading.Lock()
        self.get_logger().info(f"started usb node and created publisher")
        self.timer = self.create_timer(0.1, self._timer_callback)
        self.usb_center = Float64MultiArray()
        self.usb_center.data = [0.0, 0.0]
        self.H_inv = None
        #wreciever args
        self.frame_store = FrameStore()
        self.stop_event = threading.Event()
        self.connection_timeout = 10.0
        self.window_prefix = "Stream"
        self.discovery_port = 5550
        discovery_result = discover_stream_config(self.discovery_port, self.connection_timeout, streamer_name_filter=None)
        self.broadcast_ip = discovery_result["streamer_ip"]
        self.stream_count = discovery_result["stream_count"]
        self.base_port = discovery_result["base_port"]
        self.ports = [self.base_port + i for i in range(self.stream_count)]
        threading.Thread(target = receive_camera_data, args=(self.broadcast_ip,self.ports[0], self.connection_timeout, self.frame_store, self.stop_event), daemon=True).start()
        
        
        


    def _timer_callback(self):
        """periodic timer to resend current key if we're waiting for arm response"""
        
        #get frame 
        frame = self.frame_store.get_frame(self.window_prefix + f"-{self.ports[0]}")
        if frame is None:
            self.get_logger().debug("Did not get frame from store")
            return
        img = frame.copy()
        img = cv2.rotate(img, cv2.ROTATE_180)
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        enhanced = preprocess_for_aruco(frame)
        corners, ids, rejected_corners = cv2.aruco.detectMarkers(enhanced, ARUCO_DICT, parameters=ARUCO_PARAMS)
        self.get_logger().info(f"Detected - {ids} markers")
        self.get_logger().info(f"Corners - {corners}")
        rejected_corners = list(rejected_corners)
        
        # throw out false positives from keycap icons / LED labels while constricting by known
        if ids is not None:
            MIN_AREA = 100
            MIN_ASPECT = 0.6
            MAX_ASPECT = 1.4
            mask = []
            for i, c in enumerate(corners):
                if ids[i][0] not in [1, 2, 3, 4]:
                    #self.get_logger().info(f"Ignored - {ids[i]} - INVALID OPTION")
                    rejected_corners.append(c)
                    continue
                pts = c.reshape(4, 2)
                area = cv2.contourArea(pts)
                # if area < MIN_AREA:
                #     self.get_logger().info(f"Ignored - {ids[i]} - TOO SMALL AREA")
                #     rejected_corners.append(c)
                #     continue
                x, y, w, h = cv2.boundingRect(pts)
                if h == 0:
                    #self.get_logger().info(f"Ignored - {ids[i]} - ZERO HEIGHT")
                    rejected_corners.append(c)
                    continue
                aspect = w / h
                if aspect < MIN_ASPECT or aspect > MAX_ASPECT:
                    #self.get_logger().info(f"Ignored - {ids[i]} - aspect outside range - aspect - {aspect}")
                    rejected_corners.append(c)
                    continue
                mask.append(i)
            corners = [corners[i] for i in mask]
            ids = ids[mask] if len(mask) > 0 else None
    
        # try to get a fresh homography for center 
        H, self.src_pts = compute_homography(corners, ids)
        #self.get_logger().info(f"1 - SRC PTS - {self.src_pts}")
        # Example: src_points = np.array([[x1, y1], [x2, y2], [x3, y3], [x4, y4]])
        
        self.get_logger().info(f"Center: {self.key_center}")
        self.H_inv = None
        if H is not None and self.src_pts is not None:
            try:
                self.H_inv = np.linalg.inv(H)
            except np.linalg.LinAlgError:
                pass
            
        # use fresh if we got one, otherwise fall back to cache
        if self.H_inv is not None:
            self.cached_H_inv = self.H_inv
            self.cached_src = self.src_pts
            self.last_good = time.time()
        elif self.cached_H_inv is not None and (time.time() - self.last_good) < LOCK_TIMEOUT:
            self.H_inv = self.cached_H_inv
            self.src_pts = self.cached_src
        else:
            self.cached_H_inv = None
            
        #This implementation gets center using the homography and the known keyboard center,
        #should be more redundant to missing markers 
        if self.H_inv is not None:
            # H_inv maps mm -> pixels (correct direction)
            cx_px, cy_px = mm_to_px(self.H_inv, USB_W / 2.0, USB_D / 2.0)
            #TODO may need new manual correction for usb task 
            x_center = cx_px - 640.0 + 280  #Manual correction
            y_center = 480.0 / 2.0 - cy_px - 86
            self.get_logger().info(f"Center: {x_center:.1f}, {y_center:.1f}")
            self.usb_center.data = [float(x_center), float(y_center)]
            self.usb_center_pub.publish(self.usb_center)    
        
        #self.get_logger().info(f"Made it past center, H_inv val - {self.H_inv}")
        
        # status bar
        if self.H_inv is not None:
            draw_keys(img, self.H_inv)
            age = time.time() - self.last_good
            if age < 0.1:
                status = f"TRACKING (LIVE) | {len(ids)} markers"
            else:
                status = f"TRACKING (LOCKED) {age:.1f}s) | {len(ids)} markers"
            color = (0, 255, 0)
        else:
            n = 0 if ids is None else len(ids)
            if n >= 4:
                status = "4 markers found but homography failed (bad angle?)"
            else:
                status = f"Need 4 markers (found {n})"
            color = (0, 0, 255)
        #self.get_logger().info(f"2 - SRC PTS - {self.src_pts}, ids - {ids}")
        
        draw_ids(img, ids, corners, self.src_pts, rejected_corners = None)
        cv2.putText(img, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)
        cv2.imshow("Keyboard ArUco", img)
        k = cv2.waitKey(1) & 0xFF
            
            
            
            
def main(args=None):
   
    rclpy.init(args=args)
    node = UsbNode()
    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()
            
            
