import laspy
import numpy as np


def load_and_clean_lidar(path, voxel_size=0.35, min_points_per_cell=2):
    las_data = laspy.read(path)

    x = np.asarray(las_data.x)
    y = np.asarray(las_data.y)
    z = np.asarray(las_data.z)

    # Stack into (N, 3) array — each row is one LiDAR point
    points = np.vstack((x, y, z)).T

    # VOXEL HASH FILTER
    # Assign every point to a voxel cell using integer grid coordinates.
    # This is O(N) — one pass, no KD-tree, no pairwise distance computation.
    cell_x = np.floor(x / voxel_size).astype(np.int64)
    cell_y = np.floor(y / voxel_size).astype(np.int64)

    # Encode (cell_x, cell_y) as a single integer key for fast grouping.
    # Shift both axes to start from 0, then use the actual y range as the
    # row stride — this guarantees no collisions regardless of terrain size
    # or coordinate system (UTM northings can be ~4,200,000m so cell_y values
    # can reach ~12,000,000 — a fixed multiplier like 1,000,000 would collide).
    cell_x_shifted = cell_x - cell_x.min()
    cell_y_shifted = cell_y - cell_y.min()
    y_range = int(cell_y_shifted.max()) + 1
    keys = cell_x_shifted * y_range + cell_y_shifted

    # Sort points by cell key so all points in the same cell are contiguous
    sort_idx = np.argsort(keys, kind='stable')
    keys_sorted = keys[sort_idx]
    points_sorted = points[sort_idx]

    # Find where each new cell starts
    cell_starts = np.flatnonzero(np.diff(keys_sorted, prepend=keys_sorted[0] - 1))
    cell_ends = np.append(cell_starts[1:], len(keys_sorted))
    cell_counts = cell_ends - cell_starts

    # OUTLIER REMOVAL
    # Drop cells with fewer than min_points_per_cell points.
    # Sparse cells are almost always noise or edge artifacts — same effect as
    # statistical outlier removal but O(N) instead of O(N log N).
    valid_mask = cell_counts >= min_points_per_cell
    valid_starts = cell_starts[valid_mask]
    valid_ends = cell_ends[valid_mask]

    # REPRESENTATIVE POINT SELECTION
    # For each valid cell, keep the point with the highest Z value.
    # This is a real measured LiDAR point (not a centroid), and represents
    # the actual terrain surface the rover will encounter.
    # Using max Z avoids underestimating slopes — centroid averaging would
    # make steep cells appear less severe than they really are.
    kept_indices = np.array([
        valid_starts[i] + np.argmax(points_sorted[valid_starts[i]:valid_ends[i], 2])
        for i in range(len(valid_starts))
    ])

    return points_sorted[kept_indices]
