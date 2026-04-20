import sys
import os
import numpy as np

# Add the parent folder of project_root to Python's path
# so that imports like "from src.utils..." resolve correctly
sys.path.append(os.path.dirname(os.path.dirname(__file__)))

from src.pointcloud.load_clean import load_and_clean_lidar
from src.pointcloud.knn_builder import build_knn
from src.pathplanning.graph_builder import build_graph_vectorized
from src.utils.nearest_point import find_nearest_node
from src.utils.point_conversion import get_epsg, gps_to_xy
from src.pathplanning.multi_target import compute_cost_matrix, find_best_order, build_full_path
from src.utils.visualization import visualize_path

def main():
    # Get the LiDAR file path from the user and load it
    # load_and_clean_lidar removes outliers and voxel downsamples the point cloud
    file_path = input("Enter file path : ").strip()
    points = load_and_clean_lidar(file_path)

    # Build the KDTree and KNN structure from the cleaned point cloud
    # neighbour_indices[i] -> indices of K nearest neighbors of point i
    # neighbour_distances[i] -> their corresponding euclidean distances
    # tree -> the KDTree used for fast spatial queries
    neighbour_indices, neighbour_distances, tree = build_knn(points)

    # Build the weighted graph before finding nearest nodes
    # Must be done first so we can check if nodes are connected when snapping GPS points
    # graph[i] -> list of (neighbour_index, edge_weight) for node i
    # Edges with slope > MAX_SLOPE_DEG are rejected
    graph = build_graph_vectorized(points, neighbour_indices, neighbour_distances)

    # Read EPSG once from the LAZ file and reuse for all GPS conversions
    epsg = get_epsg(file_path)

    # Take GPS input for the start point and convert to XY projected coordinates
    # Only lat/lon is taken — z comes from the LiDAR terrain data via nearest node snapping
    lat, lon = map(float, input("Enter gps info of starting point (lat lon) : ").split())
    x, y = gps_to_xy(lat, lon, epsg)

    # Snap the start GPS point to the nearest connected node in the graph using 2D distance
    # start_idx -> index of the nearest valid node in points array
    # start_nearest_point -> 3D coordinates of that node
    start_idx, start_nearest_point = find_nearest_node([x, y], points, graph)

    # Take the number of target points from the user
    NT = int(input("Enter number of targets : "))
    target_nodes = []

    for i in range(NT):
        # Take GPS input for each target and convert to XY projected coordinates
        lat, lon = map(float, input(f"Enter gps info of {i+1}th target (lat lon) : ").split())
        x, y = gps_to_xy(lat, lon, epsg)
        # Snap each target GPS point to the nearest connected node in the graph using 2D distance
        nearest_idx, nearest_point = find_nearest_node([x, y], points, graph)
        # Store each target as a dictionary with its graph index and 3D coordinates
        target_nodes.append({"index": int(nearest_idx), "point": nearest_point.tolist()})

    # position 0 is always start, positions 1..N are targets
    # this is the list that cost_matrix and path_matrix are indexed against
    node_indices = [start_idx] + [t["index"] for t in target_nodes]

    # runs A* between every pair of nodes in node_indices
    # cost_matrix[i][j] -> total travel cost from node i to node j
    # path_matrix[i][j] -> actual list of graph node indices from node i to node j
    print("Computing pairwise paths, this may take a moment...")
    cost_matrix, path_matrix = compute_cost_matrix(graph, points, node_indices)

    # brute forces all permutations of target visit order using the cost matrix
    # best_order -> list of matrix positions in optimal visit order e.g. [2, 1, 3]
    # best_cost -> total cost of that optimal ordering
    best_order, best_cost = find_best_order(cost_matrix)
    print(f"Best path cost : {best_cost:.2f}")

    # stitches together individual A* path segments in the best order
    # returns a single flat list of node indices representing the complete route
    full_path = build_full_path(path_matrix, best_order)
    print(f"Full path has {len(full_path)} nodes")

    # --- PATH SLOPE ANALYSIS ---
    # Compute the slope between every consecutive pair of nodes on the path.
    # This lets us objectively compare two different pipeline outputs:
    # lower max slope = safer for the rover.
    if len(full_path) > 1:
        p = points[full_path]
        dx = np.diff(p[:, 0])
        dy = np.diff(p[:, 1])
        dz = np.diff(p[:, 2])
        horizontal = np.sqrt(dx**2 + dy**2)
        horizontal = np.maximum(horizontal, 1e-6)
        slopes_deg = np.degrees(np.arctan(np.abs(dz) / horizontal))
        print(f"Path slope — max: {slopes_deg.max():.1f}°  "
              f"mean: {slopes_deg.mean():.1f}°  "
              f"segments over 20°: {(slopes_deg > 20).sum()}")

    # run visualization automatically, opens as an HTML file in the browser
    visualize_path(full_path, points, start_idx, target_nodes, epsg)

    # return full_path and points together since visualization will need both
    return full_path, points

if __name__ == "__main__":
    main()