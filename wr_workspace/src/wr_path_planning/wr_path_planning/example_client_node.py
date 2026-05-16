import rclpy
from rclpy.node import Node 
from wr_interfaces.srv import PathPlan, MultiPathPlan
from geographic_msgs.msg import GeoPoint

from typing import List

class ExampleClientNode(Node):
    def __init__(self):
        super().__init__("example_client")
        self.single_path_cli = self.create_client(PathPlan, 'path_plan')
        self.multi_path_cli = self.create_client(MultiPathPlan, 'multi_path_plan')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Path planner service not available, waiting again...')
        

    def send_find_path_request(self, start: GeoPoint, goal: GeoPoint):
        req = PathPlan.Request()
        req.start = start
        req.goal = goal
        future = self.single_path_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
    
    def send_find_multi_path_request(self, start: GeoPoint, targets: List[GeoPoint]):
        req = MultiPathPlan.Request()
        req.start = start
        req.targets = targets
        future = self.multi_path_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main(args=None):
    rclpy.init(args)
    client_node = ExampleClientNode()

    # Single path request
    start = GeoPoint()
    start.latitude = 38.39186706186518
    start.longitude = -110.78002712470159

    goal = GeoPoint()
    goal.latitude = 38.40282260510612
    goal.longitude = -110.7749701640673

    result = client_node.send_find_path_request(start, goal)
    points: List[GeoPoint] = result.path
    for i, point in enumerate(points):
        client_node.get_logger().info(f"Point {i}: {(point.latitude, point.longitude)}")
    
    # Multi path request
    start = GeoPoint()
    start.latitude = 38.39925417620823
    start.longitude = -110.79526161409899

    targets: List[GeoPoint] = []
    coordinates = [(38.39436392568703, -110.77396953294046), (38.40191346263155, -110.77705301027112), (38.391980270297964, -110.78313025044322), (38.406535310086106, -110.8048434636758)]
    for lat, lon in coordinates:
        target = GeoPoint()
        target.latitude = lat
        target.longitude = lon
        targets.append(target)
    
    result = client_node.send_find_multi_path_request(start, targets)
    targets_ordered: List[GeoPoint] = result.targets_ordered
    for i, target in enumerate(targets_ordered):
        client_node.get_logger().info(f"Target {i + 1}): {(target.latitude, target.longitude)}")
    
    path: List[GeoPoint] = result.path
    client_node.get_logger().info(f"Found the following path to follow. Number of points on the path: {len(path)}")
    for i in range(0, len(path)):
        point = path[i]
        client_node.get_logger().info(f"Point {i}: {(point.latitude, point.longitude)}")
    
    client_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
