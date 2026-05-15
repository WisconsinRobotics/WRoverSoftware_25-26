import rclpy
from rclpy.node import Node 
from wr_interfaces.srv import PathPlan
from geographic_msgs.msg import GeoPoint

class ExampleClientNode(Node):
    def __init__(self):
        super().__init__("example_client")
        self.cli = self.create_client(PathPlan, 'path_plan')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Path planner service not available, waiting again...')
        

    def send_request(self, start: GeoPoint, goal: GeoPoint):
        req = PathPlan.request()
        req.start = start
        req.goal = goal
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main(args=None):
    rclpy.init(args)

    # Sample start and goal points
    start = GeoPoint()
    start.latitude = 38.39186706186518
    start.longitude = -110.78002712470159

    goal = GeoPoint()
    goal.latitude = 38.40282260510612
    goal.longitude = -110.7749701640673

    client_node = ExampleClientNode()
    result = client_node.send_request(start, goal)
    points = result.path
    for i, point in enumerate(points):
        client_node.get_logger().info(f"Point {i}: {(point.latitude, point.longitude)}")
    
    client_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
