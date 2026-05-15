import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray



class StateMachine(node):
    def __init__(self):
        # Initialization
        super().__init__('state_machine')
        
        # Create waypoint publisher
        self.waypoint_publisher = self.create_publisher(Float32MultiArray, 'waypoint', 10)
        
        # Timer callback at 20 Hz
        self.timer = self.create_timer(0.05, self.timer_callback)
        
    
    
    def timer_callback(self):
        if self.state == "START":
            self.start()
            
        elif self.state == "PLANNING":
            self.planning()
            
        elif self.state == "NAV":
            self.nav()
            
        elif self.state == "ARUCO_NAV":
            self.aruco_nav()
            
        elif self.state == "OBJECT_NAV":
            self.object_nav()
            
        elif self.state == "DANCE_OFF":
            self.dance_off()
            
        elif self.state == "END":
            self.get_logger().info("Autonomous mission complete")
            self.destroy_node()
            rclpy.shutdown()
           
           
           
    def start(self):
    
    
    
    def planning(self):
    
    
    
    def nav(self):
    
    
    
    def aruco_nav(self):
    
    
    
    def objecy_nav(self):
    
    
    
    def dance_off(self):
    
        
        
def main(args=None):
    rclpy.init(args=args)
    
    publisher = StateMachine()
    
    rclpy.spin(publisher)
    
    publisher.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
