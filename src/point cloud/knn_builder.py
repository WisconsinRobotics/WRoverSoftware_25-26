import numpy as np
from configs.settings import K_NEIGHBORS
from scipy.spatial import cKDTree


def build_knn(points):
    # build kdtree, kd-tree is a spatial data struct allows to efficiently find nearest neighbour
    # instead of checking every pair of points the kd-tree organises them in a binary space partitioning struct
    # this makes nearest neighbour queries fast from O(N^2) to O(logN)
    tree = cKDTree(points)

    # we use (K_NEIGHBOURS+1) because when you ask for neighbours, the first one returned is the point itself
    # for each point i
    # indices[i] -> indies of the 11 closest points
    # distances[i] -> contains their corresponding euclidean distances
    distances, indices = tree.query(points, k=K_NEIGHBORS + 1, workers=-1)

    # remove self neighbour
    neighbour_indices = indices[:, 1:] # shape -> (N,k)
    neighbour_distances = distances[:, 1:] # shape -> (N,k)

    return neighbour_indices, neighbour_distances, tree
