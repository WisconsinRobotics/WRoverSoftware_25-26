# Import the relevent node packages
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# PYSERIAL
import serial
# Threading
import threading
from concurrent.futures import ThreadPoolExecutor
#other
import time
import json


"""
Class responsible for getting, processing, and sending out battery data to subscribers.
This will use serial port via a USB in order to communicate with the ESP32
"""

class BatteryDataPublisher(Node):
    """
    Constructor for BatteryDataPublisher
    """
    def _init_(self):
        # Init node class stuff
        super()._init_("BatteryDataPublisher")
        
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
    # def timer_callback(self):
    #     # Create the Message to send out   
    #     msg = String()    

    #     if(self.serial_data_lock.acquire(blocking = False)):
    #         self.data = self.serial_data
    #         self.serial_data_lock.release()

    #     msg.data = "data : " + self.data

    #     # Publish out the message
    #     self.publisher_.publish(msg)
    #     self.get_logger().info("Publishing: " + msg.data)
    #     pass

    def timer_callback(self):
        msg = String()

        if self.serial_data_lock.acquire(blocking=False):
            self.data = self.serial_data
            self.serial_data_lock.release()

        # change dict in string JSON
        if isinstance(self.data, dict):
            data_str = json.dumps(self.data)
        else:
            data_str = str(self.data)

        msg.data = "data : " + data_str

        self.publisher_.publish(msg)
        self.get_logger().info("Publishing: " + msg.data)


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
    # def get_serial_worker(self):
    #     self.get_logger().info("Starting serial worker")
    #     # While the loop is allowed to do stuff, try to read the data on the serial bus
    #     counter = 0

    #     while not self.serial_event_loop.is_set():
    #         # Write the data if we get any
    #         counter += 1 
            
    #         if(counter % 10 == 0):
    #             with self.serial_data_lock:
    #                 self.serial_data = f"{counter}"
    #         pass
    #         # inside of here, if we get a full set of data, write to data
    #         # use a buffer here. Write to the buffer each time we actually have data
    #         # if we find an escape sequence in the buffer, add the data, remove that part from the buffer
    #         # should probably use a bytearray()
            
    #     pass


    def get_serial_worker(self):
        self.get_logger().info("Starting serial worker")

        # buffer start
        buffer = ""
        start_marker = '\x02' #start and end of the message
        end_marker = '\x03'

        try:
            # real serial connection
            #self.serial = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)

            #emulation
            self.serial = serial.Serial('/tmp/ttyV0', 115200, timeout=0.1)


            self.get_logger().info("Serial port opened successfully")
        except Exception as e:
            self.get_logger().error(f"Erro ao abrir serial: {e}")
            return

        while not self.serial_event_loop.is_set():
            try:
                
                if self.serial.in_waiting > 0:
                    data = self.serial.read(self.serial.in_waiting).decode('utf-8', errors='ignore')
                    buffer += data

                    # Processing messages
                    while start_marker in buffer and end_marker in buffer:
                        start = buffer.find(start_marker)
                        end = buffer.find(end_marker, start)

                        if start != -1 and end != -1 and end > start:
                            json_str = buffer[start + 1:end]
                            buffer = buffer[end + 1:]

                            try:
                                payload = json.loads(json_str) #dict we can change this type of you want

                                with self.serial_data_lock:
                                    self.serial_data = payload

                                v = payload.get("voltage", 0)
                                soc = payload.get("soc", 0)
                                self.get_logger().info(f"Battery: {v:.2f} V | SoC: {soc:.1f}%")

                            except json.JSONDecodeError:
                                self.get_logger().warn(f"Erro ao decodificar JSON: {json_str}")

                time.sleep(0.1)

            except Exception as e:
                self.get_logger().error(f"Erro no serial worker: {e}")
                time.sleep(1)



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


if _name_ == '_main_':
    main()