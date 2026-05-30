import rclpy
import threading
from rclpy.node import Node
from std_msgs.msg import String

class StateMachineControllerNode(Node):
    def __init__(self):
        # Initialization and create publisher
        super().__init__('state_machine_controller')
        self.publisher = self.create_publisher(String, '/state_machine_controller', 10)
        
        # Start a thread for console input
        self.thread = threading.Thread(target=self.get_user_input, daemon=True)
        self.thread.start()
        
    def get_user_input(self):
        print("Available Commands: continue, stop, skip (go to next target), next (go to next point)")
        print("Continue: continue the autonomous mission")
        print("Stop: stop autonomous mission and enter manual control mode")
        print("Skip: after stopping, skip the current target and start navigating to to the next target")
        
        # Get user commands
        while rclpy.ok():
            cmd = input("Enter command: ")
            
            if cmd in ['continue', 'stop', 'skip', 'next']:
                msg = String()
                msg.data = cmd
                self.publisher.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    
    state_machine_controller = StateMachineControllerNode()
    
    rclpy.spin(state_machine_controller)
    
    state_machine_controller.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
