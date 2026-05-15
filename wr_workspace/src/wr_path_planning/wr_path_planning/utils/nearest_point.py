import numpy as np
from typing import Tuple, List
from wr_path_planning.configs.settings import K_NEIGHBORS


def find_nearest_node(xy: List[float], points_array: np.ndarray, graph) -> Tuple[int, np.ndarray]:
    """Finds the closest vertex using 2D XY distance."""

    xy = np.asarray(xy, dtype=float)  # shape (2,)

    # Compute 2D horizontal distances
    dists_2d = np.linalg.norm(points_array[:, :2] - xy, axis=1)

    # Get K nearest candidates
    candidate_indices = np.argpartition(dists_2d, K_NEIGHBORS)[:K_NEIGHBORS]
    candidate_indices = candidate_indices[np.argsort(dists_2d[candidate_indices])]

    # Check valid graph nodes
    for idx in candidate_indices:
        if idx < len(graph) and len(graph[idx]) > 0:
            return idx, points_array[idx]

    # Fallback
    idx = candidate_indices[0]
    return idx, points_array[idx]