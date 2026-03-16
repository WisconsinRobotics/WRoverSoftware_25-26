# keyboard aruco detection node
# 4 ArUco markers on K552 corners -> homography -> publish movement vectors to arm
# arm_response topic triggers next key in sequence
# L for launch word Q to quit P to print
#TODO add hardcoded movement vectors
#TODO add in path correction, eyeball global offset first and try to use that, otherwise tuning and transforming

#Path idea 1: add a global offset if the arm misses in one direction consistently.
#Path idea 2: Use a per region correction tuning step if the accuracy changes at the edges for example.
#Press 4-5 keys across board, measure error, and then add affine transformation?
#Path idea 3: Pynput but I would only measure error on a diff key but would close state machine loop but dont think
# will be usable in comp so prolly not

import cv2
import numpy as np
import json
import time
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# K552 dimensions

KEYBOARD_W = 354.0
KEYBOARD_D = 123.0
PITCH = 19.05  # standard 1u key spacing
KB_CX = KEYBOARD_W / 2.0  # arm home position
KB_CY = KEYBOARD_D / 2.0
Z_MM = 10.0  # default press depth need to be tuned

# Movement vectors TODO measure and hard code
# dx+ is right, dy+ is up.

MOVES = {
    'A': (0.0, 0.0),
    'B': (0.0, 0.0),
    'C': (0.0, 0.0),
    'D': (0.0, 0.0),
    'E': (0.0, 0.0),
    'F': (0.0, 0.0),
    'G': (0.0, 0.0),
    'H': (0.0, 0.0),
    'I': (0.0, 0.0),
    'J': (0.0, 0.0),
    'K': (0.0, 0.0),
    'L': (0.0, 0.0),
    'M': (0.0, 0.0),
    'N': (0.0, 0.0),
    'O': (0.0, 0.0),
    'P': (0.0, 0.0),
    'Q': (0.0, 0.0),
    'R': (0.0, 0.0),
    'S': (0.0, 0.0),
    'T': (0.0, 0.0),
    'U': (0.0, 0.0),
    'V': (0.0, 0.0),
    'W': (0.0, 0.0),
    'X': (0.0, 0.0),
    'Y': (0.0, 0.0),
    'Z': (0.0, 0.0),
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
    'Space': (0.0, 0.0),
    'Enter': (0.0, 0.0),
}

# overlay needs absolute positions, so convert back from center-relative
WIDTHS = {
    'Space': PITCH * 6.25,
    'Enter': PITCH * 2.25,
    'Bksp':  PITCH * 2.0,
    'Tab':   PITCH * 1.5,
}
#Use for abs positioning for debugging
KEYS = {}
for label, (dx, dy) in MOVES.items():
    KEYS[label] = {
        'cx_mm': KB_CX + dx,
        'cy_mm': KB_CY + dy,
        'w_mm': WIDTHS.get(label, PITCH),
        'h_mm': PITCH,
    }


#aruco setup
ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
ARUCO_PARAMS = cv2.aruco.DetectorParameters()
# dst for homography
KB_CORNERS_MM = np.array([
    [0.0, 0.0],
    [KEYBOARD_W, 0.0],
    [KEYBOARD_W, KEYBOARD_D],
    [0.0, KEYBOARD_D],
], dtype=np.float32)


#aruco detection helpers
#average to center pixel
def marker_center(one_corner):
    pts = one_corner.reshape(4, 2)
    return pts.mean(axis=0).astype(np.float32)

#inside vertex for h
def inside_corner(one_corner, all_centers):
    pts = one_corner.reshape(4, 2)
    centroid = np.mean(all_centers, axis=0)
    dists = np.linalg.norm(pts - centroid, axis=1)
    return pts[np.argmin(dists)].astype(np.float32)

#Decide which arucos are what
def sort_four_corners(centers):
    pts = np.array(centers, dtype=np.float32)
    by_y = np.argsort(pts[:, 1])
    top, bot = by_y[:2], by_y[2:]
    tl = top[np.argmin(pts[top, 0])]
    tr = top[np.argmax(pts[top, 0])]
    bl = bot[np.argmin(pts[bot, 0])]
    br = bot[np.argmax(pts[bot, 0])]
    return [tl, tr, br, bl]

#detects, sorts, takes inside corners, then compute h with these
def compute_homography(corners, ids):
    """detect -> sort -> inside corners -> H matrix (pixels to mm)"""
    if ids is None or len(corners) < 4:
        return None, None
    corners_v = list(corners[:4])
    centers = [marker_center(c) for c in corners_v]
    order = sort_four_corners(centers)
    src = np.array([
        inside_corner(corners_v[order[i]], centers)
        for i in range(4)
    ], dtype=np.float32)
    H, _ = cv2.findHomography(src, KB_CORNERS_MM)
    return H, src

#Keyboard mm to pixels
def mm_to_px(H_inv, x_mm, y_mm):
    pt = np.array([[[x_mm, y_mm]]], dtype=np.float32)
    out = cv2.perspectiveTransform(pt, H_inv)
    return int(out[0, 0, 0]), int(out[0, 0, 1])

def char_to_label(ch):
    up = ch.upper()
    if up in MOVES:
        return up
    return {' ': 'Space', '\n': 'Enter', '\t': 'Tab'}.get(ch)

#Drawing(only for debugging overlay, could remove in final)

def draw_keys(frame, H_inv):
    h, w = frame.shape[:2]
    for label, k in KEYS.items():
        cx, cy = k['cx_mm'], k['cy_mm']
        hw, hh = k['w_mm'] / 2.0, k['h_mm'] / 2.0
        cpx, cpy = mm_to_px(H_inv, cx, cy)
        tl = mm_to_px(H_inv, cx - hw, cy - hh)
        br = mm_to_px(H_inv, cx + hw, cy + hh)
        if br[0] < 0 or br[1] < 0 or tl[0] > w or tl[1] > h:
            continue
        cv2.rectangle(frame, tl, br, (255, 165, 0), 1)
        cv2.circle(frame, (cpx, cpy), 2, (0, 255, 255), -1)
        key_w_px = abs(br[0] - tl[0])
        if key_w_px > 12:
            fs = max(0.25, min(0.38, key_w_px / 65.0))
            cv2.putText(frame, label, (tl[0] + 2, tl[1] + 10),
                        cv2.FONT_HERSHEY_SIMPLEX, fs, (255, 165, 0), 1)


def draw_arrow(frame, H_inv, key_label):
    """red arrow from keyboard center to target key"""
    if key_label not in KEYS:
        return
    k = KEYS[key_label]
    start = mm_to_px(H_inv, KB_CX, KB_CY)
    end = mm_to_px(H_inv, k['cx_mm'], k['cy_mm'])
    cv2.arrowedLine(frame, start, end, (0, 0, 255), 3, tipLength=0.04)
    cv2.circle(frame, start, 6, (0, 255, 255), -1)
    hw, hh = k['w_mm'] / 2.0, k['h_mm'] / 2.0
    tl = mm_to_px(H_inv, k['cx_mm'] - hw, k['cy_mm'] - hh)
    br = mm_to_px(H_inv, k['cx_mm'] + hw, k['cy_mm'] + hh)
    cv2.rectangle(frame, tl, br, (0, 255, 0), 2)
    cv2.putText(frame, f"-> {key_label}", (end[0] + 10, end[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)


def draw_markers(frame, corners, ids, sorted_src):
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
    # red dots at the inside corners used for homography
    if sorted_src is not None:
        for pt, name in zip(sorted_src, ['TL', 'TR', 'BR', 'BL']):
            cv2.circle(frame, (int(pt[0]), int(pt[1])), 6, (0, 0, 255), -1)
            cv2.putText(frame, name, (int(pt[0]) + 8, int(pt[1]) + 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)


# ROS2 node
#publishes to keyboard_movement with key, dx_mm, dy_mm, z_mm
#subscribes to arm_response with ANY message meaning send next key

class KeyboardNode(Node):

    def __init__(self):
        super().__init__('keyboard_aruco')
        self.movement_pub = self.create_publisher(String, 'keyboard_movement', 10)
        # Need to store sub to protect against not get garbage collected
        self._arm_sub = self.create_subscription(String, 'arm_response', self._on_arm_response, 10)
        self._keys = []
        self._idx = 0
        self._waiting = False  # sent command, waiting for arm
        self._lock = threading.Lock()  # spin thread writes, main thread reads
        self.get_logger().info(f"keyboard_aruco has {len(MOVES)} keys")

    def start_sequence(self, word, keys):
        with self._lock:
            self._keys = keys
            self._idx = 0
            self._waiting = False
        self.get_logger().info(f"Sequence should be {word} -> {keys}")
        self._publish_current()

    def _publish_current(self):
        with self._lock:
            if self._idx >= len(self._keys):
                self.get_logger().info("Done.")
                return
            label = self._keys[self._idx]
            self._waiting = True
        dx, dy = MOVES[label]
        msg = String()
        msg.data = json.dumps({
            'key': label,
            'dx_mm': dx,
            'dy_mm': dy,
            'z_mm': Z_MM,
        })
        self.movement_pub.publish(msg)
        self.get_logger().info(f"Sent: {label} dx={dx:.1f} dy={dy:.1f}")
    #When arm done publish next
    def _on_arm_response(self, msg):
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


# ── Main ───────────────────────────────────────────────────────────────────

def prompt_word():
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


def main():
    rclpy.init()
    node = KeyboardNode()

    cap = cv2.VideoCapture(1)
    if not cap.isOpened():
        node.get_logger().error("Couldn't open webcam index 1")
        node.destroy_node()
        rclpy.shutdown()
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    # spin ros2 in background so arm_response callback fires
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # cache last good homography for 5s so hand blocking a marker doesnt kill tracking
    cached_H_inv = None
    cached_src = None
    last_good = 0.0
    LOCK_TIMEOUT = 5.0

    print(f"Keyboard node with {len(MOVES)} keys")
    print("Q=quit  L=launchkey  P=print\n")

    while rclpy.ok():
        ret, frame = cap.read()
        if not ret:
            break

        img = frame.copy()
        corners, ids, _ = cv2.aruco.detectMarkers(img, ARUCO_DICT, parameters=ARUCO_PARAMS)
        H, src_pts = compute_homography(corners, ids)

        # invert H (pixels->mm) to get H_inv (mm->pixels) for drawing
        H_inv = None
        if H is not None and src_pts is not None:
            try:
                H_inv = np.linalg.inv(H)
            except np.linalg.LinAlgError:
                pass

        # update or use cache
        if H_inv is not None:
            cached_H_inv = H_inv
            cached_src = src_pts
            last_good = time.time()
        elif cached_H_inv is not None and (time.time() - last_good) < LOCK_TIMEOUT:
            H_inv = cached_H_inv
            src_pts = cached_src
        else:
            cached_H_inv = None

        if H_inv is not None:
            draw_keys(img, H_inv)
            age = time.time() - last_good
            if age < 0.1:
                status = f"TRACKING (LIVE) | {len(KEYS)} keys"
            else:
                status = f"TRACKING (LOCKED {age:.1f}s) | {len(KEYS)} keys"
            color = (0, 255, 0)
        else:
            n = 0 if ids is None else len(ids)
            if n >= 4:
                status = f"4 markers found but homography failed (bad angle?)"
            else:
                status = f"Need 4 markers (found {n})"
            color = (0, 0, 255)

        draw_markers(img, corners, ids, src_pts)

        # show arrow to current target key during a sequence
        if node.active and H_inv is not None:
            current = node.current_key
            if current:
                draw_arrow(img, H_inv, current)
                cv2.putText(img, f"Typing: {current}", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.putText(img, status, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)
        cv2.imshow("Keyboard ArUco", img)

        k = cv2.waitKey(1) & 0xFF
        if k == ord('q'):
            break
        elif k == ord('p'):
            for lbl, (dx, dy) in MOVES.items():
                print(f"  {lbl:8s} dx={dx:7.1f} dy={dy:7.1f}")
        elif k == ord('l'):
            if H_inv is None:
                print("No tracking - need 4 markers visible")
            else:
                word, keys = prompt_word()
                if word and keys:
                    node.start_sequence(word, keys)

    cap.release()
    cv2.destroyAllWindows()
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == "__main__":
    main()