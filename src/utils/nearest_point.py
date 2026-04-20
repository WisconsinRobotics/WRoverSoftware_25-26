import numpy as np
from configs.settings import K_NEIGHBORS  # K_NEIGHBORS = 10, how many candidates to check

def find_nearest_node(xy, points_array, graph):
    # GPS gives us x/y only — altitude comes from the terrain (LiDAR), not from GPS input.
    # So we snap using 2D horizontal distance only, ignoring z entirely.
    # We query the 3D KDTree using the 2D distances computed against x/y columns only.
    xy = np.asarray(xy)  # [x, y]

    # Compute 2D horizontal distances from xy to every point in the cloud
    # points_array[:, :2] extracts only the x/y columns
    dists_2d = np.linalg.norm(points_array[:, :2] - xy, axis=1)

    # Get indices of K closest points by 2D distance
    candidate_indices = np.argpartition(dists_2d, K_NEIGHBORS)[:K_NEIGHBORS]
    # Sort them closest to furthest
    candidate_indices = candidate_indices[np.argsort(dists_2d[candidate_indices])]

    # Iterate through candidates from closest to furthest
    for idx in candidate_indices:
        # Check 1: node exists in the graph
        # Check 2: node has at least one edge (not an isolated dead-end node)
        # A node can exist in points_array but have no edges if all its
        # connections were rejected due to being too steep in graph_builder
        if idx < len(graph) and len(graph[idx]) > 0:
            # Return the index and the 3D coordinates of the chosen node
            return idx, points_array[idx]

    # Fallback: if all K candidates are isolated nodes (extremely unlikely),
    # just return the geometrically closest one rather than crashing
    return candidate_indices[0], points_array[candidate_indices[0]]