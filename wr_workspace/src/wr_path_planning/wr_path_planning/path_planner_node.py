import rclpy
from rclpy.node import Node 
from wr_interfaces.srv import PathPlan
from geographic_msgs.msg import GeoPoint

from wr_path_planning.point_cloud.load_clean import load_and_clean_lidar
from wr_path_planning.point_cloud.knn_builder import build_knn
from wr_path_planning.path_planning.graph_builder import build_graph_vectorized
from wr_path_planning.path_planning.astar import astar
from wr_path_planning.utils.nearest_point import find_nearest_node
from wr_path_planning.utils.point_conversion import *

import random

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
        self.espg = get_epsg(self.lidar_file)
        self.get_logger().info(f"EPSG of the lidar file: {self.espg}")
        self.points = load_and_clean_lidar(self.lidar_file, logger=self.get_logger())

        self.get_logger().info("Constructing an adjacency graph from LiDAR data")
        neighbour_indices, neighbour_distances, _ = build_knn(self.points)
        self.graph = build_graph_vectorized(self.points, neighbour_indices, neighbour_distances)
        
        # Testing the service in test mode
        self.declare_parameter("test_mode", False)
        self.test_mode = self.get_parameter("test_mode").get_parameter_value().bool_value
        if self.test_mode:
            # Pick two random points in lidar dataset
            i, j = random.sample(range(len(self.points)), 2)
            start_x, start_y = self.points[i][0], self.points[i][1]
            goal_x, goal_y = self.points[j][0], self.points[j][1]

            start = GeoPoint()
            start.longitude, start.latitude = xy_to_gps(start_x, start_y, self.espg)
            goal = GeoPoint()
            goal.longitude, goal.latitude = xy_to_gps(goal_x, goal_y, self.espg)
            self.get_logger().info(f"Entering test mode. Start: {(start.latitude, start.longitude)}; Goal: {(goal.latitude, goal.longitude)}")

            request = PathPlan.Request()
            request.start = start
            request.goal = goal
            response = PathPlan.Response()
            self.find_path_callback(request, response)

            points = response.path
            self.get_logger().info(f"Found the following path to follow. Number of points on the path: {len(points)}")
            for i in range(0, len(points)):
                point = points[i]
                self.get_logger().info(f"Point {i}: {(point.latitude, point.longitude)}")

        if self.test_mode:
            self.get_logger().info("Finished building and testing the graph. Starting the server")
        else:
            self.get_logger().info("Finished building the graph. Starting the server")
        self.srv = self.create_service(PathPlan, "path_plan", self.find_path_callback)
        

    def find_path_callback(self, request, response):
        start = request.start
        goal = request.goal 

        # Convert points to x, y
        start_lon, start_lat = start.longitude, start.latitude
        start_x, start_y = gps_to_xy(start_lon, start_lat, self.espg)

        goal_lon, goal_lat = goal.longitude, goal.latitude
        goal_x, goal_y = gps_to_xy(goal_lon, goal_lat, self.espg)

        # Find indices in our graph that are closest to start and end
        start_idx, _ = find_nearest_node((start_x, start_y), self.points, self.graph)
        goal_idx, _ = find_nearest_node((goal_x, goal_y), self.points, self.graph)
        
        # Run Astar algorithmn and convert points to GeoPoint format
        path_indixes, _ = astar(self.graph, self.points, start_idx, goal_idx)
        path = []
        for indx in path_indixes:
            x, y, altitude = self.points[indx]
            lon, lat = xy_to_gps(x, y, self.espg)

            point = GeoPoint()
            point.latitude = lat
            point.longitude = lon
            point.altitude = altitude

            path.append(point)


        # Package data
        response.path = path
        response.message = ""
        response.success = True



def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
