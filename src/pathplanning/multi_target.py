import itertools
from pathplanning.astar import astar

def compute_cost_matrix(graph, points, node_indices):
    
    # node_indices is a list like [start_idx, T1_idx, T2_idx, ..., TN_idx]
    # n is the total number of nodes we care about (start + all targets)
    n = len(node_indices)

    # Initialize both matrices with None
    # cost_matrix[i][j] will hold the travel cost from node i to node j
    # path_matrix[i][j] will hold the actual path (list of node indices)
    cost_matrix = [[None] * n for _ in range(n)]
    path_matrix = [[None] * n for _ in range(n)]

    # We only compute upper triangle (i < j) since graph is bidirectional
    # cost i->j == cost j->i, so we mirror it to fill the full matrix
    for i in range(n):
        for j in range(i + 1, n):

            # Get the actual graph node indices for position i and j
            src = node_indices[i]
            dst = node_indices[j]

            # Run A* between them
            path, cost = astar(graph, points, src, dst)

            # Fill both directions since travel cost is symmetric
            cost_matrix[i][j] = cost
            cost_matrix[j][i] = cost

            path_matrix[i][j] = path
            # Reverse the path for the opposite direction
            path_matrix[j][i] = path[::-1] if path else None

    return cost_matrix, path_matrix


def find_best_order(cost_matrix):

    # Number of targets is everything except the start (index 0)
    # So target positions in our matrix are indices 1, 2, ..., n-1
    n = len(cost_matrix)
    target_positions = list(range(1, n))

    best_cost = float('inf')
    best_order = None

    if len(target_positions) > 10:
        raise ValueError(f"Too many targets ({len(target_positions)}) for brute-force TSP. Max is 10.")

    # Try every possible ordering of the targets
    # itertools.permutations handles any N without hardcoding
    for perm in itertools.permutations(target_positions):

        # perm is one possible ordering, e.g. (2, 3, 1) meaning visit T2, T3, T1
        # Always start from index 0 (the start node)
        cost = 0
        prev = 0
        reachable = True

        for curr in perm:
            leg_cost = cost_matrix[prev][curr]
            if leg_cost == float('inf'):
                reachable = False
                break
            cost += leg_cost
            prev = curr

        if reachable and cost < best_cost:
            best_cost = cost
            best_order = perm

    if best_order is None:
        raise ValueError("No valid path exists between all waypoints. Some nodes may be unreachable.")

    return list(best_order), best_cost


def build_full_path(path_matrix, best_order):

    # best_order is something like [2, 3, 1] meaning
    # visit target at matrix position 2, then 3, then 1
    # We always start from matrix position 0 (start node)
    full_path = []
    prev = 0

    for curr in best_order:
        segment = path_matrix[prev][curr]

        if segment is None:
            raise ValueError(f"No path found between nodes {prev} and {curr}")

        # Avoid duplicating the connecting node between segments
        # The last node of one segment is the first node of the next
        if full_path:
            segment = segment[1:]

        full_path.extend(segment)
        prev = curr

    return full_path
