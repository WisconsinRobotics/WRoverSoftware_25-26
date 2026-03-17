# Import the relevent node packages
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# PYSERIAL
import serial
import serial.tools.list_ports

# Threading
import threading
from concurrent.futures import ThreadPoolExecutor

#other
import time
import json
import struct

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
        #self.serial_event_loop.set()
        self.last_time = self.get_clock().now()
        self.get_logger().info(f"{self.last_time}")

        #Start up the serial port as a property?  
        self.start_marker = bytes([0xAA])
        self.end_marker = bytes([0x55])
        self.buffer = bytearray()

        
        # Start up the worker thread
        self.serial_worker = threading.Thread(target = self.get_serial_worker)
        self.serial_worker.start()

        # Make publisher, this will publish strings of JSON files
        self.publisher_ = self.create_publisher(String, "battery_telemetry", 10)

        # timer between publishes
        self.timer_period = 0.1 # this is in seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        pass

    """
    Ros event loop that will send out data to publishers    
    """
    def timer_callback(self):
        msg = String()
        current_time = self.get_clock().now()
        delta = (current_time - self.last_time).nanoseconds / 1e9

        # pega dados do buffer thread-safe
        if self.serial_data_lock.acquire(blocking=False):
            self.data = self.serial_data
            self.serial_data_lock.release()

        if isinstance(self.data, dict):
            data_str = json.dumps(self.data)
        else:
            data_str = str(self.data)

        msg.data = f"data: {data_str} | delta time: {delta:.3f}s"
        self.publisher_.publish(msg)
        self.get_logger().info("Publishing: " + msg.data)
        self.last_time = current_time


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
        
        # close the serial ports in order to close off resources!
        print("closing serial connections...")
        # ser.close()

        print("------- battery_data_publisher cleanup finished ------")
    
    """
    This will go through the serial data and try to read shit
    """
    def get_serial_worker(self):
        try:
            self.serial = serial.Serial("/dev/ttyUSB0", 9600, timeout=0.1)
            self.get_logger().info("opened serial port")
        except Exception as e:
            self.get_logger().error(f"Error opening serial: {e}")
            return
        
        while not self.serial_event_loop.is_set():
            try:
                if self.serial.in_waiting > 0:
                    data = self.serial.read(self.serial.in_waiting)
                    self.buffer += data

                    while self.start_marker in self.buffer and self.end_marker in self.buffer:
                        start = self.buffer.find(self.start_marker)
                        end = self.buffer.find(self.end_marker, start)

                        if start != -1 and end != -1 and end > start:
                            packet = self.buffer[start : end + 1]   # pega os 15 bytes do pacote
                            self.buffer = self.buffer[end + 1:]     # remove do buffer

                            decoded = self.decode_serial(packet)
                            if decoded:
                                with self.serial_data_lock:
                                    self.serial_data = decoded

            except Exception as e:
                self.get_logger().error(f"Serial worker error: {e}")
                time.sleep(0.1)

    """
    Method that will take a start and end position on the buffer and try to decode that data.
    @return -1 if corrupted data, data struct if the checksum is the same.
    
    # This will decode a 15 byte packet from ESP32
    # BYTE 0 = start byte
    # BYTE 1 - 4 = Voltage
    # BYTE 5 - 8 = current
    # BTYE 9 - 12 = Amp hours
    # BYTE 13 = XOR checksum
    # BYTE 14 = end byte
    # ----------
    # The decoded struct will look like this
    # [0] = start byte
    # [1] = voltage
    # [2] = current
    # [3] = amp hours
    # [4] = XOR checksum
    # [5] = end
    """
    def decode_serial(self, packet: bytes):
        if len(packet) != 15:
            return None  # tamanho errado → lixo

        # Verifica start/end
        if packet[0] != 0xAA or packet[-1] != 0x55:
            return None

        # Checa checksum (bytes 1–12, que são os floats)
        data_bytes = packet[1:13]
        checksum = 0
        for b in data_bytes:
            checksum ^= b

        if checksum != packet[13]:
            return None  # corrompido

        # Desempacota 3 floats
        voltage, current, ampHours = struct.unpack("<fff", data_bytes)

        return {
            "voltage": voltage,
            "current": current,
            "ampHours": ampHours
        }



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
