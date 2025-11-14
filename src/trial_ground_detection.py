import cv2
import numpy as np
from typing import List

def detect_ground_points(frame_with_depth_values) -> List[List]:

    floor_threshold_depth = 0.1# To be figured out experimentally
    depth = frame_with_depth_values.astype(np.float32)

    # If depth is RGB, convert to 1-channel depth magnitude
    if depth.ndim == 3 and depth.shape[2] == 3:
        depth_gray = np.linalg.norm(depth, axis=2)
    else:
        depth_gray = depth

    H, W = depth_gray.shape
    ground_points = []

    # Compute vertical absolute difference for entire image at once
    # diff[y,x] = |depth[y,x] - depth[y-1,x]|
    diff = np.abs(depth_gray[1:, :] - depth_gray[:-1, :])

    # Scan each column bottom-to-top
    for x in range(W):
        col = diff[:, x]  # vector of length H-1
        idx = np.where(col > floor_threshold_depth)[0]

        if len(idx) > 0:
            y = idx[0] + 1  # +1 because diff starts at y=1
            ground_points.append((y, x))

    # Convert to NumPy array if needed
    ground_points = np.array(ground_points)
    # for point in range(len(ground_points) - 2):
    #     depth = cv2.line(depth, ground_points[point], ground_points[point + 1], color=(0, 0, 255), thickness=10)
    return ground_points
    # return depth


def draw_line_segments(image, segments, color=(0, 0, 255), thickness=2):

    # Make a copy so original isn't modified
    output = image.copy()
    number_of_segments = len(segments) - 1
    for i, seg in enumerate(segments):
        if i < number_of_segments:

            (y1, x1), (y2, x2) = seg, segments[i + 1]
        else:
            continue
        cv2.line(output, (x1, y1), (x2, y2), color, thickness)

    return output