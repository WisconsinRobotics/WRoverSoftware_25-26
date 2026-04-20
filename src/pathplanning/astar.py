import heapq
import numpy as np

def astar(graph, points, start_idx, goal_idx):
    # HEURISTIC
    # Straight line 3D euclidean distance from node n to the goal
    # This guides A* toward the goal efficiently
    # It never overestimates real cost, so A* stays optimal
    def heuristic(n):
        return np.linalg.norm(points[n] - points[goal_idx])

    # OPEN SET
    # This is the priority queue (min-heap) of nodes to explore
    # Each entry is (f_score, node_index)
    # f = g + h, where g = cost so far, h = estimated cost to goal
    # heapq always pops the entry with the lowest f_score first
    open_set = []
    heapq.heappush(open_set, (0, start_idx))

    # G SCORE
    # g_score[n] = the actual cost of the cheapest path found so far from start to n
    # Everything starts as infinity (undiscovered)
    # Start node costs 0 to reach
    g_score = {start_idx: 0.0}

    # CAME FROM
    # came_from[n] = which node we came from to reach n on the best path
    # Used at the end to reconstruct the full path by backtracking
    came_from = {}

    # VISITED SET
    # Once a node is popped from the heap and processed, we mark it visited
    # We never need to process a node twice
    visited = set()

    while open_set:

        # Pop the node with the lowest f_score
        f, current = heapq.heappop(open_set)

        # If we've reached the goal, reconstruct and return the path
        if current == goal_idx:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start_idx)
            path.reverse()
            return path, g_score[goal_idx]

        # Skip if already processed
        if current in visited:
            continue
        visited.add(current)

        # EXPLORE NEIGHBORS
        for neighbor, weight in graph[current]:

            if neighbor in visited:
                continue

            # Tentative g score if we go through current to reach neighbor
            tentative_g = g_score[current] + weight

            # Only update if this path to neighbor is cheaper than any previously found
            if tentative_g < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score = tentative_g + heuristic(neighbor)
                heapq.heappush(open_set, (f_score, neighbor))

    # If we exhaust the open set without reaching the goal, no path exists
    return None, float('inf')
