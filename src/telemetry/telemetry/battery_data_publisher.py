# Import the relevent node packages
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


"""
Class responsible for getting, processing, and sending out battery data to subscribers.
This will use serial port via a USB in order to communicate with the:q

"""
class BatteryDataPublisher(Node):
    #Basic construstructor
    def __init__(self):
        pass

    def timer_callback(self):
        pass




def main():
    print('Hi from telemetry.')


if __name__ == '__main__':
    main()
