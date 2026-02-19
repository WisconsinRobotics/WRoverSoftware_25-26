#!/usr/bin/env python3
# keyboard_aruco_laptop_test.py
#
# Drop-in test version of keyboard_aruco_node.py that uses a laptop webcam
# instead of a ROS image topic. No DepthAI needed. No ROS needed.
#
# Run: python3 keyboard_aruco_laptop_test.py
# Press Q to quit.
#
# When all 4 markers (IDs 0,1,2,3) are visible, the homography is computed
# and printed — same logic as the real node. Green overlay = detected,
# blue dot = marker center, red text = mm coordinates on keyboard plane.

import cv2
import numpy as np

# ── Keyboard dimensions (mm) ─────────────────────────────────────────────────
KEYBOARD_WIDTH_MM  = 354.0
KEYBOARD_DEPTH_MM  = 123.0

# Marker corner positions in keyboard space (mm). Origin = top-left.
# 0=top-left, 1=top-right, 2=bottom-right, 3=bottom-left
KEYBOARD_MARKER_POSITIONS_MM = {
    0: (0.0,               0.0),
    1: (KEYBOARD_WIDTH_MM, 0.0),
    2: (KEYBOARD_WIDTH_MM, KEYBOARD_DEPTH_MM),
    3: (0.0,               KEYBOARD_DEPTH_MM),
}

# ── Camera intrinsics (DepthAI OAK — swap for your laptop if you have them) ──
# For laptop testing the distortion barely matters for homography, so
# we leave undistortion optional (set UNDISTORT = False to skip).
UNDISTORT = False

K = np.array([
    [569.7166137695312, 0.0,              622.4732666015625],
    [0.0,              569.48486328125,   367.3853454589844],
    [0.0,              0.0,              1.0],
], dtype=np.float64)

DISTORTION = np.array([
    2.9425814151763916, 0.7698521018028259, -3.687290518428199e-05,
    0.00017509849567431957, 0.00862385705113411, 3.298475503921509,
    1.6266201734542847, 0.09254294633865356,
    0.0, 0.0, 0.0, 0.0,
    -0.0020870917942374945, 0.004025725182145834,
], dtype=np.float64)

# ── ArUco setup ───────────────────────────────────────────────────────────────
ARUCO_DICT   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
ARUCO_PARAMS = cv2.aruco.DetectorParameters()


def marker_center(corners_one):
    """Return (cx, cy) float32 for one marker's corners array."""
    pts = corners_one.reshape((4, 2))
    return np.array([pts[:, 0].mean(), pts[:, 1].mean()], dtype=np.float32)


def pixel_to_keyboard_mm(H, u, v):
    """Map pixel (u,v) → (x_mm, y_mm) on keyboard using homography H."""
    pt  = np.array([[[u, v]]], dtype=np.float32)
    out = cv2.perspectiveTransform(pt, H)
    return float(out[0, 0, 0]), float(out[0, 0, 1])


def compute_homography(corners, ids):
    """
    Given detected corners + ids, try to compute the pixel→keyboard homography.
    Returns H (3x3) if all 4 markers found, else None.
    """
    if ids is None or len(ids) == 0:
        return None

    ids_flat = ids.flatten()
    id_to_pt = {}
    for i, mid in enumerate(ids_flat):
        mid = int(mid)
        if mid in KEYBOARD_MARKER_POSITIONS_MM:
            id_to_pt[mid] = marker_center(corners[i])

    if set(id_to_pt.keys()) != {0, 1, 2, 3}:
        return None

    src = np.array([id_to_pt[i] for i in range(4)], dtype=np.float32)
    dst = np.array([KEYBOARD_MARKER_POSITIONS_MM[i] for i in range(4)], dtype=np.float32)

    H, _ = cv2.findHomography(src, dst)
    return H


def draw_overlay(frame, corners, ids, H):
    """Draw detection overlay and, if H is valid, show mm coords."""
    if ids is None:
        return

    ids_flat = ids.flatten()
    for i, mid in enumerate(ids_flat):
        pts = corners[i].reshape((4, 2)).astype(np.int32)
        cv2.polylines(frame, [pts], isClosed=True, color=(0, 255, 0), thickness=2)

        cx, cy = marker_center(corners[i])
        cv2.circle(frame, (int(cx), int(cy)), 5, (255, 0, 0), -1)

        label = f"ID {int(mid)}"
        if H is not None and int(mid) in KEYBOARD_MARKER_POSITIONS_MM:
            xmm, ymm = pixel_to_keyboard_mm(H, cx, cy)
            label += f"  ({xmm:.1f}, {ymm:.1f}) mm"

        cv2.putText(frame, label,
                    (pts[0][0], pts[0][1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)


def main():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Could not open webcam at index 0. Try changing to 1 or 2.")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    print("Keyboard ArUco laptop test — press Q to quit")
    print(f"Looking for marker IDs: 0, 1, 2, 3  (DICT_4X4_50)")
    print(f"Keyboard plane: {KEYBOARD_WIDTH_MM} x {KEYBOARD_DEPTH_MM} mm")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame.")
            break

        img = frame.copy()
        if UNDISTORT:
            img = cv2.undistort(img, K, DISTORTION)

        corners, ids, _ = cv2.aruco.detectMarkers(img, ARUCO_DICT, parameters=ARUCO_PARAMS)

        H = compute_homography(corners, ids)

        if H is not None:
            status = "HOMOGRAPHY OK — all 4 markers visible"
            color  = (0, 255, 0)
            # Print to terminal (mirrors what the ROS node logs)
            print(f"Homography:\n{H}")
        else:
            found = [] if ids is None else [int(x) for x in ids.flatten()
                                             if int(x) in KEYBOARD_MARKER_POSITIONS_MM]
            missing = [i for i in range(4) if i not in found]
            status = f"Waiting for markers — missing IDs: {missing}"
            color  = (0, 0, 255)

        draw_overlay(img, corners, ids, H)
        cv2.putText(img, status, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

        cv2.imshow("Keyboard ArUco Test (laptop cam)", img)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
