#!/usr/bin/env python3
# keyboard_aruco_laptop_test.py
#
# Laptop webcam test for keyboard detection. Uses 4 ArUco markers
# at the corners of a Redragon K552 to compute homography and project
# key positions onto the camera feed. Press L to type a word and see
# the movement vectors drawn from center to each key.
#
# Not a ROS2 node yet, just testing with the webcam.
#
# Q = quit, L = launch word sequence, P = print key positions

import cv2
import numpy as np
import time

# keyboard dimensions (Redragon K552 TKL, standard ANSI)
KEYBOARD_WIDTH_MM = 354.0
KEYBOARD_DEPTH_MM = 123.0

KEY_PITCH = 19.05  # standard 1u key spacing
KEY_H = KEY_PITCH

# row y-centers measured from top of keyboard
ROW_Y = {
    'fn':   6.5,
    'num':  25.55,
    'tab':  44.60,
    'caps': 63.65,
    'shft': 82.70,
    'spc':  101.75,
}

# each entry: (label, cx_mm, cy_mm, width_mm)
KEY_LAYOUT = []

def u(n):
    return n * KEY_PITCH

def add_row(keys, y, start_x):
    x = start_x
    for label, width_units in keys:
        w = u(width_units)
        KEY_LAYOUT.append((label, x + w / 2.0, y, w))
        x += w

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
add_row([
    ('`',1), ('1',1), ('2',1), ('3',1), ('4',1), ('5',1),
    ('6',1), ('7',1), ('8',1), ('9',1), ('0',1), ('-',1),
    ('=',1), ('Bksp',2),
], ROW_Y['num'], 4.5)
for i, name in enumerate(['Ins', 'Home', 'PgUp']):
    KEY_LAYOUT.append((name, u(14.75) + 4.5 + i * u(1.0) + u(0.5), ROW_Y['num'], u(1.0)))

# QWERTY row
add_row([
    ('Tab',1.5), ('Q',1), ('W',1), ('E',1), ('R',1), ('T',1),
    ('Y',1), ('U',1), ('I',1), ('O',1), ('P',1),
    ('[',1), (']',1), ('\\',1.5),
], ROW_Y['tab'], 4.5)
for i, name in enumerate(['Del', 'End', 'PgDn']):
    KEY_LAYOUT.append((name, u(14.75) + 4.5 + i * u(1.0) + u(0.5), ROW_Y['tab'], u(1.0)))

# home row
add_row([
    ('Caps',1.75), ('A',1), ('S',1), ('D',1), ('F',1), ('G',1),
    ('H',1), ('J',1), ('K',1), ('L',1), (';',1), ("'",1),
    ('Enter',2.25),
], ROW_Y['caps'], 4.5)

# shift row
add_row([
    ('LShft',2.25), ('Z',1), ('X',1), ('C',1), ('V',1), ('B',1),
    ('N',1), ('M',1), (',',1), ('.',1), ('/',1), ('RShft',2.75),
], ROW_Y['shft'], 4.5)
KEY_LAYOUT.append(('Up', u(14.75) + 4.5 + u(1.5), ROW_Y['shft'], u(1.0)))

# bottom row
add_row([
    ('LCtrl',1.25), ('LWin',1.25), ('LAlt',1.25), ('Space',6.25),
    ('RAlt',1.25), ('Fn',1.25), ('RCtrl',1.25),
], ROW_Y['spc'], 4.5)
KEY_LAYOUT.append(('Left',  u(14.75) + 4.5 + u(0.5), ROW_Y['spc'], u(1.0)))
KEY_LAYOUT.append(('Down',  u(14.75) + 4.5 + u(1.5), ROW_Y['spc'], u(1.0)))
KEY_LAYOUT.append(('Right', u(14.75) + 4.5 + u(2.5), ROW_Y['spc'], u(1.0)))

# dict for quick lookups
KEY_MAP = {}
for lbl, cx, cy, w in KEY_LAYOUT:
    KEY_MAP[lbl] = {'cx_mm': cx, 'cy_mm': cy, 'w_mm': w, 'h_mm': KEY_H}

# center of the keyboard (arm home position)
KB_CENTER_X = KEYBOARD_WIDTH_MM / 2.0
KB_CENTER_Y = KEYBOARD_DEPTH_MM / 2.0


# --- launchpad ---
# tracks the word being displayed as vectors

class LaunchpadState:
    def __init__(self):
        self.active = False
        self.word = ""
        self.keys = []
        self.current_idx = 0
        self.start_time = 0.0
        self.hold_time = 2.5  # seconds per vector

    def start(self, word, keys):
        self.active = True
        self.word = word
        self.keys = keys
        self.current_idx = 0
        self.start_time = time.time()

    def tick(self):
        """returns current key label, or None when sequence is done"""
        if not self.active:
            return None
        if len(self.keys) == 0:
            self.active = False
            return None

        elapsed = time.time() - self.start_time
        if elapsed >= self.hold_time:
            self.current_idx += 1
            self.start_time = time.time()
            if self.current_idx >= len(self.keys):
                self.active = False
                print("Sequence done.")
                return None

        return self.keys[self.current_idx]

    def get_progress_text(self):
        if not self.active:
            return ""
        return f"[{self.current_idx + 1}/{len(self.keys)}] {self.word} -> '{self.keys[self.current_idx]}'"

launchpad = LaunchpadState()


# --- camera calibration (from realsense, not used for laptop cam) ---

USE_UNDISTORT = False

CAMERA_MATRIX = np.array([
    [569.7166137695312, 0.0, 622.4732666015625],
    [0.0, 569.48486328125, 367.3853454589844],
    [0.0, 0.0, 1.0],
], dtype=np.float64)

DIST_COEFFS = np.array([
    2.9425814151763916, 0.7698521018028259, -3.687290518428199e-05,
    0.00017509849567431957, 0.00862385705113411, 3.298475503921509,
    1.6266201734542847, 0.09254294633865356,
    0.0, 0.0, 0.0, 0.0, -0.0020870917942374945, 0.004025725182145834,
], dtype=np.float64)


# --- aruco ---

ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
ARUCO_PARAMS = cv2.aruco.DetectorParameters()

# corners in mm space (TL, TR, BR, BL)
KEYBOARD_CORNERS_MM = np.array([
    [0.0, 0.0],
    [KEYBOARD_WIDTH_MM, 0.0],
    [KEYBOARD_WIDTH_MM, KEYBOARD_DEPTH_MM],
    [0.0, KEYBOARD_DEPTH_MM],
], dtype=np.float32)


# --- helper functions ---

def get_marker_center(corner):
    pts = corner.reshape((4, 2))
    return np.array([pts[:, 0].mean(), pts[:, 1].mean()], dtype=np.float32)

def sort_corners(centers):
    """sort 4 marker centers into TL TR BR BL order geometrically"""
    pts = np.array(centers, dtype=np.float32)
    sorted_by_y = np.argsort(pts[:, 1])
    top_two = pts[sorted_by_y[:2]]
    bot_two = pts[sorted_by_y[2:]]
    tl = top_two[np.argmin(top_two[:, 0])]
    tr = top_two[np.argmax(top_two[:, 0])]
    bl = bot_two[np.argmin(bot_two[:, 0])]
    br = bot_two[np.argmax(bot_two[:, 0])]
    return np.array([tl, tr, br, bl], dtype=np.float32)

def find_homography(corners, ids):
    """returns (H, src_pts) or (None, None) if less than 4 markers"""
    if ids is None or len(ids) < 4:
        return None, None
    centers = [get_marker_center(corners[i]) for i in range(min(4, len(corners)))]
    src = sort_corners(centers)
    H, _ = cv2.findHomography(src, KEYBOARD_CORNERS_MM)
    return H, src

def mm_to_px(H_inv, x_mm, y_mm):
    pt = np.array([[[x_mm, y_mm]]], dtype=np.float32)
    result = cv2.perspectiveTransform(pt, H_inv)
    return (int(result[0, 0, 0]), int(result[0, 0, 1]))

def char_to_key_label(ch):
    """maps a typed char to KEY_MAP label. returns None if not found."""
    upper = ch.upper()
    if upper in KEY_MAP:
        return upper
    special = {' ': 'Space', '\n': 'Enter', '\t': 'Tab'}
    if ch in special and special[ch] in KEY_MAP:
        return special[ch]
    return None


# --- drawing ---

def draw_all_keys(frame, H_inv):
    h, w = frame.shape[:2]
    for label, info in KEY_MAP.items():
        cx, cy = info['cx_mm'], info['cy_mm']
        hw, hh = info['w_mm'] / 2.0, info['h_mm'] / 2.0

        cx_px, cy_px = mm_to_px(H_inv, cx, cy)
        x1, y1 = mm_to_px(H_inv, cx - hw, cy - hh)
        x2, y2 = mm_to_px(H_inv, cx + hw, cy + hh)

        if x2 < 0 or y2 < 0 or x1 > w or y1 > h:
            continue

        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 165, 0), 1)
        cv2.circle(frame, (cx_px, cy_px), 2, (0, 255, 255), -1)

        key_w_px = abs(x2 - x1)
        if key_w_px > 12:
            fs = max(0.25, min(0.38, key_w_px / 65.0))
            cv2.putText(frame, label, (x1 + 2, y1 + 10),
                        cv2.FONT_HERSHEY_SIMPLEX, fs, (255, 165, 0), 1)

def draw_vector_arrow(frame, H_inv, key_label):
    """draws arrow from keyboard center to target key"""
    if key_label not in KEY_MAP:
        return
    info = KEY_MAP[key_label]

    start = mm_to_px(H_inv, KB_CENTER_X, KB_CENTER_Y)
    end = mm_to_px(H_inv, info['cx_mm'], info['cy_mm'])

    cv2.arrowedLine(frame, start, end, (0, 0, 255), 3, tipLength=0.04)
    cv2.circle(frame, start, 6, (0, 255, 255), -1)

    # highlight target key
    hw, hh = info['w_mm'] / 2.0, info['h_mm'] / 2.0
    tl = mm_to_px(H_inv, info['cx_mm'] - hw, info['cy_mm'] - hh)
    br = mm_to_px(H_inv, info['cx_mm'] + hw, info['cy_mm'] + hh)
    cv2.rectangle(frame, tl, br, (0, 255, 0), 2)

    cv2.putText(frame, f"-> {key_label}", (end[0] + 10, end[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

def draw_markers(frame, corners, ids, sorted_pts):
    if ids is None:
        return
    corner_names = ["TL", "TR", "BR", "BL"]
    for i in range(len(ids)):
        pts = corners[i].reshape((4, 2)).astype(np.int32)
        cv2.polylines(frame, [pts], True, (0, 255, 0), 2)
        center = get_marker_center(corners[i])
        cv2.circle(frame, (int(center[0]), int(center[1])), 5, (255, 0, 0), -1)
        cv2.putText(frame, f"ID {int(ids[i][0])}", (pts[0][0], pts[0][1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    if sorted_pts is not None:
        for pt, name in zip(sorted_pts, corner_names):
            cv2.putText(frame, name, (int(pt[0]) + 8, int(pt[1]) + 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)


# --- word input ---

def prompt_for_word():
    """terminal prompt to enter a word. returns (word, keys) or (None, None)."""
    raw = input("\nEnter a word (up to 5 letters, e.g. ROVER): ").strip()
    if not raw:
        return None, None

    if len(raw) > 5:
        print(f"Trimming to first 5 chars.")
        raw = raw[:5]

    key_labels = []
    valid_chars = []
    for ch in raw:
        label = char_to_key_label(ch)
        if label is not None:
            key_labels.append(label)
            valid_chars.append(ch.upper())
        else:
            print(f"  skipping '{ch}' (not on keyboard)")

    if not key_labels:
        print("No valid keys.")
        return None, None

    word_str = ''.join(valid_chars)
    print(f"Sequence: {word_str} -> {key_labels}")
    return word_str, key_labels

def print_key_info():
    print(f"\nK552 key positions ({len(KEY_MAP)} keys):")
    for lbl, info in KEY_MAP.items():
        print(f"  {lbl:8s} cx={info['cx_mm']:6.1f} cy={info['cy_mm']:6.1f} "
              f"w={info['w_mm']:5.1f} h={info['h_mm']:5.1f}")


# --- main ---

def main():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Couldn't open webcam. Try index 1 or 2.")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    print(f"Keyboard ArUco test - {len(KEY_MAP)} keys mapped")
    print("Q=quit  L=launch word  P=print keys")
    print("Place 4 ArUco markers (DICT_4X4_50) at the keyboard corners.\n")

    has_printed_keys = False

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        img = frame.copy()
        if USE_UNDISTORT:
            img = cv2.undistort(img, CAMERA_MATRIX, DIST_COEFFS)

        corners, ids, _ = cv2.aruco.detectMarkers(img, ARUCO_DICT, parameters=ARUCO_PARAMS)

        H, sorted_src = find_homography(corners, ids)
        H_inv = None

        if H is not None:
            try:
                H_inv = np.linalg.inv(H)
            except np.linalg.LinAlgError:
                H_inv = None

        if H_inv is not None:
            draw_all_keys(img, H_inv)
            status = f"TRACKING | {len(KEY_MAP)} keys"
            status_color = (0, 255, 0)
            if not has_printed_keys:
                print_key_info()
                has_printed_keys = True
        else:
            n = 0 if ids is None else len(ids)
            status = f"Need 4 markers (found {n})"
            status_color = (0, 0, 255)
            has_printed_keys = False

        draw_markers(img, corners, ids, sorted_src)

        # launchpad animation
        if launchpad.active and H_inv is not None:
            current_key = launchpad.tick()
            if current_key is not None:
                draw_vector_arrow(img, H_inv, current_key)
                cv2.putText(img, launchpad.get_progress_text(), (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                cv2.putText(img, f"Word: {launchpad.word}", (10, 90),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        cv2.putText(img, status, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, status_color, 2)
        cv2.imshow("Keyboard ArUco Test", img)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('p') and H_inv is not None:
            print_key_info()
        elif key == ord('l') and H_inv is not None:
            word, keys = prompt_for_word()
            if word is not None and keys is not None:
                launchpad.start(word, keys)

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()