# keyboard aruco detection node
# 4 ArUco markers on K552 corners -> homography -> publish movement vectors to arm
# arm_response topic triggers next key in sequence
# L for launch word Q to quit P to print
#wreciever imports 
from typing import List
import zmq
import base64
import socket


import cv2
import numpy as np
import json
import time
import threading
import argparse
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from std_msgs.msg import Float64MultiArray
    HAS_ROS = True
except ImportError:
    HAS_ROS = False
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

# CLAHE helps even out glare/reflections before marker detection
CLAHE = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))


# ── aruco helpers ──────────────────────────────────────────────────────────────

def preprocess_for_aruco(frame):
    """grayscale + CLAHE so specular glare doesn't wash out markers"""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return CLAHE.apply(gray)


def marker_center(one_corner):
    """average of a marker's 4 vertices = its center"""
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
    quality = _PAIR_QUALITY.get(pair)
    return quality

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

def inside_corner(corners, center):
    """given a marker's 4 corners and center, return the corner closest to the center"""
    c = np.array(center)
    pts = corners.reshape(4, 2)
    dists = np.linalg.norm(pts - c, axis=1)
    idx = np.argmin(dists)
    return pts[idx]


# ── debug overlay drawing ─────────────────────────────────────────────────────

def draw_keys(frame, H_inv):
    """draw all key outlines + labels on the camera frame"""
    h, w = frame.shape[:2]
    for label, k in KEYS.items():
        cx, cy = k['cx_mm'], k['cy_mm']
        hw, hh = k['w_mm'] / 2.0, k['h_mm'] / 2.0

        cpx, cpy = mm_to_px(H_inv, cx, cy)
        tl = mm_to_px(H_inv, cx - hw, cy - hh)
        br = mm_to_px(H_inv, cx + hw, cy + hh)

        # skip if off screen or too small to bother
        if br[0] < 0 or br[1] < 0 or tl[0] > w or tl[1] > h:
            continue
        key_w_px = abs(br[0] - tl[0])
        key_h_px = abs(br[1] - tl[1])
        if key_w_px < 8 or key_h_px < 8:
            continue

        cv2.rectangle(frame, tl, br, (255, 165, 0), 1)
        cv2.circle(frame, (cpx, cpy), 2, (0, 255, 255), -1)

        if key_w_px > 14:
            fs = max(0.3, min(0.5, key_w_px / 50.0))
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, fs, 1)
            cv2.putText(frame, label, (cpx - tw // 2, cpy + th // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, fs, (255, 165, 0), 1)


def draw_arrow(frame, H_inv, key_label):
    """red arrow from arm home to target key, green box around target"""
    if key_label not in KEYS:
        return
    k = KEYS[key_label]
    start = mm_to_px(H_inv, KB_CX, KB_CY)
    end = mm_to_px(H_inv, k['cx_mm'], k['cy_mm'])
    hw, hh = k['w_mm'] / 2.0, k['h_mm'] / 2.0
    tl = mm_to_px(H_inv, k['cx_mm'] - hw, k['cy_mm'] - hh)
    br = mm_to_px(H_inv, k['cx_mm'] + hw, k['cy_mm'] + hh)

    cv2.arrowedLine(frame, start, end, (0, 0, 255), 3, tipLength=0.04)
    cv2.circle(frame, start, 6, (0, 255, 255), -1)
    cv2.rectangle(frame, tl, br, (0, 255, 0), 2)
    cv2.putText(frame, f"-> {key_label}", (end[0] + 10, end[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)


def draw_markers(frame, corners, ids, sorted_src):
    """draw detected marker outlines, centers, and sorted corner labels"""
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
    if sorted_src is not None:
        for pt, name in zip(sorted_src, ['TL', 'TR', 'BR', 'BL']):
            cv2.circle(frame, (int(pt[0]), int(pt[1])), 6, (0, 0, 255), -1)
            cv2.putText(frame, name, (int(pt[0]) + 8, int(pt[1]) + 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            
#wreciever helpers =======================================================   
def receive_camera_data(
	ip: str,
	port: int,
	timeout: float,
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
 
	while not first_frame_received:
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
		while True:
			try:
				frame_bytes = footage_socket.recv()
			except zmq.Again:
				continue
 
			try:
				npimg = np.frombuffer(frame_bytes, dtype=np.uint8)
				source = cv2.imdecode(npimg, cv2.IMREAD_COLOR)
				if source is None:
					continue
				cv2.imshow(window_name, source)
				if cv2.waitKey(1) & 0xFF == ord("q"):
					print("Quit signal received, exiting...")
					break
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
	receiver_socket.settimeout(DISCOVERY_SOCKET_TIMEOUT_SECONDS)

	print(
		color_text(
			f"Listening for discovery on UDP {discovery_port} for up to {timeout:.1f}s"
			f"{f' (filter={streamer_name_filter})' if streamer_name_filter else ''}...",
			ANSI_YELLOW,
		)
	)

	deadline = time.time() + max(MIN_TIMEOUT_SECONDS, timeout)
	try:
		while time.time() < deadline:
			try:
				data, _addr = receiver_socket.recvfrom(DISCOVERY_BUFFER_SIZE_BYTES)
			except socket.timeout:
				continue

			discovered = parse_discovery_payload(data, streamer_name_filter)
			if discovered is None:
				continue

			print(
				color_text(
					f"Discovered '{discovered['streamer_name']}' at {discovered['streamer_ip']} "
					f"(base_port={discovered['base_port']}, streams={discovered['stream_count']})",
					ANSI_GREEN,
				)
			)
			return discovered
	finally:
		receiver_socket.close()

	return None

    


        
# ── ROS2 node ──────────────────────────────────────────────────────────────────
# publishes JSON to keyboard_movement: {key, dx_mm, dy_mm, z_mm}
# waits for any message on arm_response before sending the next key
LOCK_TIMEOUT = 5.0
class KeyboardNode(Node):

    def __init__(self):
        super().__init__('keyboard_aruco')
        #keyboard args
        self.movement_pub = self.create_publisher(String,  'keyboard_key_positions', 10)
        self.keyboard_center_pub = self.create_publisher(Float64MultiArray, 'keyboard_center', 10)
        self._arm_sub = self.create_subscription(
            String, 'arm_response', self._on_arm_response, 10)
        self._keys = []
        self._idx = 0
        self._waiting = False
        self._lock = threading.Lock()
        self.get_logger().info(f"keyboard_aruco ready with {len(MOVES)} keys")
        self.timer = self.create_timer(0.1, self._timer_callback)
        self.frame = None
        self.cached_H_inv = None
        self.last_good = 0.0
        self.cached_H_inv = None
        self.get_logger().info("Starting frame grabber thread...")  
        self.key_center = Float64MultiArray()
        self.key_center.data = [0.0, 0.0]
        self.H_inv = None
        
        #wreciever args
    
        self.connection_timeout = 10.0
        self.window_prefix = "Stream"
        self.discovery_port = 5550
        self.broadcast_ip, self.base_port, self.stream_count = discover_stream_config(self.discovery_port, self.connection_timeout, streamer_name_filter=None)
        self.ports = [self.base_port + i for i in range(self.stream_count)]
        threading.Thread(target = receive_camera_data, args=(self.broadcast_ip,self.ports[0], self.connection_timeout))
        
        

    def _timer_callback(self):
        """periodic timer to resend current key if we're waiting for arm response"""
        frame = None
        #wreciever main loop
        with self.lock:
            keys = list(self.frames.keys())
        for k in keys:
            with self.lock:
                frame = self.frames.get(k)
            if frame is None:
                self.get_logger().warning(f"Frame for {k} disappeared")
                return
            cv2.imshow(k, frame)
                
        img = frame.copy()
        enhanced = preprocess_for_aruco(frame)
        corners, ids, _ = cv2.aruco.detectMarkers(enhanced, ARUCO_DICT, parameters=ARUCO_PARAMS)
        # throw out false positives from keycap icons / LED labels while constricting by known
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
        
        #This implementation gets center using the src points and averaging them 
        #if self.src_pts is not None and len(self.src_pts) > 0:
        #   
        #   center = np.mean(self.src_pts, axis=0)  # shape (2,)
        #    x_center, y_center = center
        #    x_center = -640/2 + x_center
        #    y_center = 480/2 - y_center
        #    
        #    self.get_logger().info(f"Center: {x_center}, {y_center}")
        #    self.key_center.data = [float(x_center), float(y_center)]
        #    self.keyboard_center_pub.publish(self.key_center)

        #self.get_logger().info(f"Center: {self.key_center}")
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
            cx_px, cy_px = mm_to_px(self.H_inv, KEYBOARD_W / 2.0, KEYBOARD_D / 2.0)
            x_center = cx_px - 640.0
            y_center = 480.0 / 2.0 - cy_px
            self.get_logger().info(f"Center: {x_center:.1f}, {y_center:.1f}")
            self.key_center.data = [float(x_center), float(y_center)]
            self.keyboard_center_pub.publish(self.key_center)    

        # status bar
        if self.H_inv is not None:
            draw_keys(img, self.H_inv)
            age = time.time() - self.last_good
            if age < 0.1:
                status = f"TRACKING (LIVE) | {len(KEYS)} keys"
            else:
                status = f"TRACKING (LOCKED {age:.1f}s) | {len(KEYS)} keys"
            color = (0, 255, 0)
        else:
            n = 0 if ids is None else len(ids)
            if n >= 4:
                status = "4 markers found but homography failed (bad angle?)"
            else:
                status = f"Need 4 markers (found {n})"
            color = (0, 0, 255)

        draw_markers(img, corners, ids, self.src_pts)

        # # show arrow to current target key if we're mid-sequence
        # if node and node.active and H_inv is not None:
        #     current = node.current_key
        #     if current:
        #         draw_arrow(img, H_inv, current)
        #         cv2.putText(img, f"Typing: {current}", (10, 60),
        #                     cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.putText(img, status, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)
        cv2.imshow("Keyboard ArUco", img)

        # keyboard controls
        k = cv2.waitKey(1) & 0xFF
     
        if k == ord('l'):
            if self.H_inv is None:
                print("No tracking — need 4 markers visible")
            else:
                word, keys = prompt_word()
                if word and keys:
                    self.start_sequence(word, keys)
           

    def start_sequence(self, word, keys):
        """kick off a new typing sequence"""
        with self._lock:
            self._keys = keys
            self._idx = 0
            self._waiting = False
        self.get_logger().info(f"Sequence: {word} -> {keys}")
        self._publish_current()

    def _publish_current(self):
        """send the current key's move vector to the arm"""
        with self._lock:
            if self._idx >= len(self._keys):
                self.get_logger().info("Sequence done.")
                return
            label = self._keys[self._idx]
            self._waiting = True

        dx, dy = MOVES[label]
        msg = String()
        msg.data = json.dumps({
            'key': label,
            'dx_mm': dx + OFFSET_X,
            'dy_mm': dy + OFFSET_Y,
            'z_mm': Z_MM,
        })
        self.movement_pub.publish(msg)
        self.get_logger().info(f"Sent: {label}  dx={dx + OFFSET_X:.1f}  dy={dy + OFFSET_Y:.1f}")
        
 

    def _on_arm_response(self, msg):
        """arm says it finished a press, advance to next key"""
        with self._lock:
            if not self._waiting or self._idx >= len(self._keys):
                return
            self._waiting = False
            self._idx += 1
        self._publish_current()

    @property
    def current_key(self):
        with self._lock:
            if self._idx < len(self._keys):
                return self._keys[self._idx]
        return None

    @property
    def active(self):
        with self._lock:
            return len(self._keys) > 0 and self._idx < len(self._keys)


# ── main loop ──────────────────────────────────────────────────────────────────

def prompt_word():
    """ask user for a word, convert to key labels (max 5 chars)"""
    raw = input("\nEnter launchkey: ").strip()
    if not raw:
        return None, None
    raw = raw[:5]
    labels, chars = [], []
    for ch in raw:
        lbl = char_to_label(ch)
        if lbl:
            labels.append(lbl)
            chars.append(ch.upper())
        else:
            print(f"  skipping '{ch}'")
    if not labels:
        return None, None
    word = ''.join(chars)
    print(f"Sequence: {word} -> {labels}")
    return word, labels


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

    node = KeyboardNode()
    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

    print(f"Keyboard node with {len(MOVES)} keys")
    print("Q=quit  L=launchkey  P=print\n")





if __name__ == "__main__":
    main()