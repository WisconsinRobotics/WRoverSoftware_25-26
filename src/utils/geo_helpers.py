import numpy as np
from configs.settings import MAX_SLOPE_DEG, SLOPE_MULTIPLIER

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

def compute_edge_weight_vectorized(i_indices, j_indices, dists, points):
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
