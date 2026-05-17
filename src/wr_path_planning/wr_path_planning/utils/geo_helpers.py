import numpy as np
from wr_path_planning.configs.settings import MAX_SLOPE_DEG, SLOPE_MULTIPLIER
from geographic_msgs.msg import GeoPoint

from typing import List, Tuple

# Weight = distance * (1 + overall slope cost)
'''
def compute_edge_weight(i, j, dist, points):
    dx = points[j][0] - points[i][0]
    dy = points[j][1] - points[i][1]
    dz = points[j][2] - points[i][2]

    horizontal = np.sqrt(dx*dx + dy*dy)
    horizontal = max(horizontal, 1e-6)

    slope_radian = np.arctan(abs(dz) / horizontal)
    slope_degree = np.degrees(slope_radian)

    if slope_degree > MAX_SLOPE_DEG:
        return None

    slope_cost = (slope_degree / MAX_SLOPE_DEG) * SLOPE_MULTIPLIER
    edge_weight = dist * (1 + slope_cost)

    return edge_weight
'''

def compute_edge_weight_vectorized(i_indices: List[int], j_indices: List[int], dists: List[float], points: List[Tuple[float]]) -> List[float]:
    """
    Compute edge weights for all edges in a vectorized manner.

    Args:
        i_indices: list of source node indices
        j_indices: list of target node indices
        dists: list of horizontal distances between nodes
        points: Nx3 array of 3D coordinates for each node
    Returns:
        edge_weights: list of edge weights, where weight is distance * (1 + slope cost)
                      If an edge is too steep, its weight is set to np.nan
    """

    dx_array = points[j_indices, 0] - points[i_indices, 0]
    dy_array = points[j_indices, 1] - points[i_indices, 1]
    dz_array = points[j_indices, 2] - points[i_indices, 2]

    horizontal_dists_array = np.sqrt(dx_array**2 + dy_array**2)
    horizontal_dists_array = np.maximum(horizontal_dists_array, 1e-6)  # avoid division by zero

    slope_radians_array = np.arctan(np.abs(dz_array) / horizontal_dists_array)
    slope_degrees_array = np.degrees(slope_radians_array)

    mask = slope_degrees_array <= MAX_SLOPE_DEG
    
    slope_costs_array = (slope_degrees_array / MAX_SLOPE_DEG) * SLOPE_MULTIPLIER
    edge_weight = dists * (1 + slope_costs_array)

    edge_weight[~mask] = np.nan

    return edge_weight

def compute_gnss_distance(a: GeoPoint, b: GeoPoint):
    """Computes distance between two GNSS points using Harvesine formula

    Args:
        a - first gnss point
        b - second gnss point
    Returns:
        Distance between two GNSS points in meters using spherical Earth model
    """
    a_lon, a_lat = a.longitude, a.latitude
    b_lon, b_lat = b.longitude, b.latitude

    d_lat = np.radians(b_lat - a_lat)
    d_lon = np.radians(b_lon - a_lon)

    a_lat_rad = np.radians(a_lat)
    b_lat_rad = np.radians(b_lat)
    
    a = np.sin(d_lat / 2)**2 + np.cos(a_lat_rad) * np.cos(b_lat_rad) * np.sin(d_lon / 2)**2
    c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1 - a))
    R = 6371000  # Earth radius in meters
    distance = R * c
    return distance
