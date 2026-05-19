import rclpy
from rclpy.node import Node 
from wr_interfaces.srv import PathPlan
from wr_interfaces.srv import MultiPathPlan
from geographic_msgs.msg import GeoPoint
from geographic_msgs.msg import GeoPath
from geographic_msgs.msg import GeoPoseStamped
from geographic_msgs.msg import GeoPose

from wr_path_planning.point_cloud.load_clean import load_and_clean_lidar
from wr_path_planning.point_cloud.knn_builder import build_knn
from wr_path_planning.path_planning.graph_builder import build_graph_vectorized
from wr_path_planning.path_planning.astar import astar
from wr_path_planning.utils.nearest_point import find_nearest_node
from wr_path_planning.utils.point_conversion import *
from wr_path_planning.utils.geo_helpers import compute_gnss_distance
from wr_path_planning.utils.point_conversion import UTAH_EPSG

import random
from itertools import permutations
from typing import List

class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner_node')
        self.get_logger().info('Path Planner Node has been started.')

        self.declare_parameter("lidar_file", "")
        self.lidar_file = self.get_parameter("lidar_file").get_parameter_value().string_value
        if not self.lidar_file:
            self.get_logger().error("No LiDAR file specified. Please set the 'lidar_file' parameter.")
            return
        
        self.get_logger().info(f"Loading and cleaning LiDAR data from: {self.lidar_file}")

        # Launch EPSG code over overwrites the epsg code of the lidar file
        self.declare_parameter("epsg", 0)
        self.epsg = self.get_parameter("epsg").get_parameter_value().integer_value
        if self.epsg == 0:
            # If not provided a launch EPSG code, try to read it from lidar file otherwise use UTAH_EPSG as an EPSG code
            self.epsg = get_epsg(self.lidar_file)
            if self.epsg is None:
                self.epsg = UTAH_EPSG

        self.get_logger().info(f"EPSG of the lidar file: {self.epsg}")
        self.points = load_and_clean_lidar(self.lidar_file, logger=self.get_logger())

        self.get_logger().info("Constructing an adjacency graph from LiDAR data")
        neighbour_indices, neighbour_distances, _ = build_knn(self.points)
        self.graph = build_graph_vectorized(self.points, neighbour_indices, neighbour_distances)
        
        # Testing the service in test mode
        self.declare_parameter("test_mode", False)
        self.test_mode = self.get_parameter("test_mode").get_parameter_value().bool_value
        if self.test_mode:
            self._test_implementation()
            self.get_logger().info("Finished building and testing the graph. Starting the server")
        else:
            self.get_logger().info("Finished building the graph. Starting the server")
        
        self.path_srv = self.create_service(PathPlan, "path_plan", self.find_path_callback)
        self.multi_path_srv = self.create_service(MultiPathPlan, "multi_path_plan", self.find_multi_path_callback)
        

    def find_path_callback(self, request, response):
        start: GeoPoint = request.start
        goal: GeoPoint = request.goal 

        # Convert points to x, y
        start_lon, start_lat = start.longitude, start.latitude
        start_x, start_y = gps_to_xy(start_lon, start_lat, self.epsg)

        goal_lon, goal_lat = goal.longitude, goal.latitude
        goal_x, goal_y = gps_to_xy(goal_lon, goal_lat, self.epsg)

        # Find indices in our graph that are closest to start and end
        start_idx, _ = find_nearest_node((start_x, start_y), self.points, self.graph)
        goal_idx, _ = find_nearest_node((goal_x, goal_y), self.points, self.graph)
        
        # Run Astar algorithmn and convert points to GeoPoint format
        path_indixes, _ = astar(self.graph, self.points, start_idx, goal_idx)
        if path_indixes is None:
            response.path = []
            response.message = "No path found between start and goal"
            response.success = False
            return response

        path = []
        for indx in path_indixes:
            x, y, altitude = self.points[indx]
            lon, lat = xy_to_gps(x, y, self.epsg)

            point = GeoPoint()
            point.latitude = lat
            point.longitude = lon
            point.altitude = altitude

            path.append(point)

        # Package data
        response.path = path
        response.message = ""
        response.success = True

        return response

    def find_multi_path_callback(self, request, response):        
        start: GeoPoint = request.start
        targets: List[GeoPoint] = request.targets

        # We are going to use very simple heuristic to order targets
        # We are going to check every possible arrangement of targets and find the arrangement that will have shortest overall path
        indexes = range(len(targets))
        best_permutation = None
        best_length = None
        for p in permutations(indexes):
            length = 0
            current_point = start
            for indx in p:
                next_point = targets[indx]
                length += compute_gnss_distance(current_point, next_point)
                current_point = next_point
            
            if best_permutation is None or length < best_length:
                best_permutation = p
                best_length = length
        
        targets_ordered: List[GeoPoint] = [targets[indx] for indx in best_permutation]
        path: List[GeoPath] = []
        current_point = start
        for target in targets_ordered:
            _request = PathPlan.Request()
            _request.start = current_point
            _request.goal = target

            _response = PathPlan.Response()
            self.find_path_callback(_request, _response)
            
            segment = GeoPath()
            for point in _response.path:
                pose = GeoPose()
                gps = GeoPoseStamped()
                pose.position = point
                gps.pose = pose
                segment.poses.append(gps)
            path.append(segment)

            current_point = target

        response.targets_ordered = targets_ordered
        response.path = path
        response.message = ""
        response.success = True 
        
        return response
    
    def _test_implementation(self):
        self.get_logger().info("Entering test mode")

        # Pick two random points in lidar dataset
        i, j = random.sample(range(len(self.points)), 2)
        start_x, start_y = self.points[i][0], self.points[i][1]
        goal_x, goal_y = self.points[j][0], self.points[j][1]

        start = GeoPoint()
        start.longitude, start.latitude = xy_to_gps(start_x, start_y, self.epsg)
        goal = GeoPoint()
        goal.longitude, goal.latitude = xy_to_gps(goal_x, goal_y, self.epsg)
        self.get_logger().info(f"Testing single target. Start: {(start.latitude, start.longitude)}; Goal: {(goal.latitude, goal.longitude)}")

        request = PathPlan.Request()
        request.start = start
        request.goal = goal
        response = PathPlan.Response()
        self.find_path_callback(request, response)

        path = response.path
        self.get_logger().info(f"Found the following path to follow. Number of points on the path: {len(path)}")
        for i in range(0, len(path)):
            point = path[i]
            self.get_logger().info(f"Point {i}: {(point.latitude, point.longitude)}")

        # Test multipath following
        indexes = random.sample(range(len(self.points)), 5)
        i = indexes[0]
        js = indexes[1:]
        
        start_x, start_y = self.points[i][0], self.points[i][1]
        start = GeoPoint()
        start.longitude, start.latitude = xy_to_gps(start_x, start_y, self.epsg)

        msg = f"Testing multi target. Start: {(start.latitude, start.longitude)}; Targets: "
        targets = []
        for i, j in enumerate(js):
            target_x, target_y = self.points[j][0], self.points[j][1]
            target = GeoPoint()
            target.longitude, target.latitude = xy_to_gps(target_x, target_y, self.epsg)
            targets.append(target)
            msg += f"{i + 1}) {(target.latitude, target.longitude)}, "

        self.get_logger().info(msg)
        request = MultiPathPlan.Request()
        request.start = start
        request.targets = targets
        response = MultiPathPlan.Response()
        self.find_multi_path_callback(request, response)

        self.get_logger().info("Found the following arrangement of targets: ")
        targets_ordered = response.targets_ordered
        for i, target in enumerate(targets_ordered):
            self.get_logger().info(f"Target {i + 1}): {(target.latitude, target.longitude)}")
        
        path: List[GeoPath] = response.path
        self.get_logger().info(f"Found the following path to follow. Number of segments on the path: {len(path)}")
        for i, segment in enumerate(path):
            self.get_logger().info(f"Segment {i + 1}")
            for j, gps in enumerate(segment.poses):
                point = gps.pose.position
                self.get_logger().info(f"Point {j}: {(point.latitude, point.longitude)}")


def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
