# Import the relevent node packages
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# PYSERIAL
import serial
# Threading
import threading
from concurrent.futures import ThreadPoolExecutor



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
        
        # Make the worker thread that gets telemetry data
        self.get_logger().info("starting up worker")
        self.serial_data = ""
        self.data = ""
        self.serial_data_lock = threading.Lock()
        self.serial_event_loop = threading.Event()
#        self.serial_event_loop.set()

        self.serial_worker = threading.Thread(target = self.get_serial_worker)
        self.serial_worker.start()


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

        if(self.serial_data_lock.acquire(blocking = False)):
            self.data = self.serial_data
            self.serial_data_lock.release()

        msg.data = "data : " + self.data

        # Publish out the message
        self.publisher_.publish(msg)
        self.get_logger().info("Publishing: " + msg.data)
        pass

    """
    Responsible for cleaning up processes on kill of the node
    """
    def on_cleanup(self):
        #self.get_logger().info("shutting down serial worker")
        print("\n\n------- battery_data_publisher cleanup start -------")
        # stop tell the event to stop reading
        print("shutting down serial workers")
        self.serial_event_loop.set()
        self.serial_worker.join()
        print("closing serial connections...")
        print("------- battery_data_publisher cleanup finished ------")
    
    """
    This will go through the serial data and try to read shit
    """
    def get_serial_worker(self):
        self.get_logger().info("Starting serial worker")
        # While the loop is allowed to do stuff, try to read the data on the serial bus
        counter = 0

        while not self.serial_event_loop.is_set():
            # Write the data if we get any
            counter += 1 
            
            if(counter % 10 == 0):
                with self.serial_data_lock:
                    self.serial_data = f"{counter}"
            pass
            # inside of here, if we get a full set of data, write to data
            # use a buffer here. Write to the buffer each time we actually have data
            # if we find an escape sequence in the buffer, add the data, remove that part from the buffer
            # should probably use a bytearray()
        pass


def main(args = None):
    # ROS STARTUP
    rclpy.init(args = args)
    battery_data_publisher = BatteryDataPublisher()

    try:
        rclpy.spin(battery_data_publisher)
    except KeyboardInterrupt:
        print("accepted keyboard interupt")
    finally:
        battery_data_publisher.on_cleanup()
        #kill the node i guess
        battery_data_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
