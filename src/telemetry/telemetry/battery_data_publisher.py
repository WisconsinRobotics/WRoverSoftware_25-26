# Import the relevent node packages
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


"""
Class responsible for getting, processing, and sending out battery data to subscribers.
This will use serial port via a USB in order to communicate with the ESP32
"""
class BatteryDataPublisher(Node):
    """
    Constructor for BatteryDataPublisher
    """
    def __init__(self):
        # Init node class stuff
        super().__init__("BatteryDataPublisher")
        
        # Make publisher, this will publish strings of JSON files
        self.publisher_ = self.create_publisher(String, "battery_telemetry", 10)

        # timer between publishes
        timer_period = 0.5 # this is in seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        pass


    """
    Ros event loop that will send out data to publishers    
    """
    def timer_callback(self):
        # Create the Message to send out   
        msg = String()    
        msg.data = "Hello World"

        # Publish out the message
        self.publisher_.publish(msg)
        self.get_logger().info("Publishing: " + msg.data)
        pass



    """
    Requests data from the serial bus and converts that into useful data.
    sends out a request 
    """
    def request_data(self):
        # We will use the PySerial Framework to read data. getting information is going
        # to be a different story
        pass

def main(args = None):
    # ROS STARTUP
    rclpy.init(args = args)

    battery_data_publisher = BatteryDataPublisher()

    rclpy.spin(battery_data_publisher)

    #kill the node i guess
    battery_datda_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
