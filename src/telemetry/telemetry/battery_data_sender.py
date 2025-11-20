# Import the relevent node packages
#ros2 topic pub /esp_commands std_msgs/msg/String '{data: "{\"voltage\": 12.5, \"current\": 3.3, \"ampHours\": 100.0}"}'

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# PYSERIAL
import serial
import serial.tools.list_ports

# Threading
import threading

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
        super().__init__("BatteryDataPublisher")

        # Thread control
        self.get_logger().info("starting up worker")
        self.serial_data = ""
        self.data = ""
        self.serial_data_lock = threading.Lock()
        self.serial_event_loop = threading.Event()
        self.last_time = self.get_clock().now()

        # Packet markers
        self.start_marker = bytes([0xAA])
        self.end_marker = bytes([0x55])
        self.buffer = bytearray()

        # Start serial worker thread
        self.serial_worker = threading.Thread(target=self.get_serial_worker)
        self.serial_worker.start()

        # Publisher → ROS2
        self.publisher_ = self.create_publisher(String, "battery_telemetry", 10)

        # Timer to publish data
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # --- NEW ---
        # Subscriber to receive commands and send to ESP32
        self.cmd_sub = self.create_subscription(
            String,
            "esp_commands",
            self.command_callback,
            10
        )

    """
    ROS timer → publishes battery telemetry
    """
    def timer_callback(self):
        msg = String()
        current_time = self.get_clock().now()
        delta = (current_time - self.last_time).nanoseconds / 1e9

        # get thread-safe data
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
    Cleanup on shutdown
    """
    def on_cleanup(self):
        print("\n\n------- battery_data_publisher cleanup start -------")
        self.serial_event_loop.set()
        self.serial_worker.join()

        if hasattr(self, "serial"):
            try:
                self.serial.close()
            except:
                pass

        print("------- battery_data_publisher cleanup finished ------")

    """
    Serial worker thread → reads incoming packets from ESP32
    """
    def get_serial_worker(self):
        try:
            self.serial = serial.Serial("/dev/ttyUSB1", 9600, timeout=0.1)
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
                            packet = self.buffer[start: end + 1]
                            self.buffer = self.buffer[end + 1:]

                            decoded = self.decode_serial(packet)
                            if decoded:
                                with self.serial_data_lock:
                                    self.serial_data = decoded

            except Exception as e:
                self.get_logger().error(f"Serial worker error: {e}")
                time.sleep(0.1)

    """
    Decode a 15-byte packet:
    [0]  = 0xAA
    [1-12] = 3 floats (12 bytes)
    [13] = checksum XOR of bytes 1–12
    [14] = 0x55
    """
    def decode_serial(self, packet: bytes):
        if len(packet) != 15:
            return None

        if packet[0] != 0xAA or packet[-1] != 0x55:
            return None

        data_bytes = packet[1:13]
        checksum = 0
        for b in data_bytes:
            checksum ^= b

        if checksum != packet[13]:
            return None

        voltage, current, ampHours = struct.unpack("<fff", data_bytes)

        return {
            "voltage": voltage,
            "current": current,
            "ampHours": ampHours
        }

    # ------------------------------
    # NEW: Send data to ESP32
    # ------------------------------
    def send_to_esp(self, voltage: float, current: float, ampHours: float):
        if not hasattr(self, "serial"):
            self.get_logger().warn("Serial not initialized yet")
            return

        try:
            # pack floats
            data_bytes = struct.pack("<fff", voltage, current, ampHours)

            # checksum
            checksum = 0
            for b in data_bytes:
                checksum ^= b

            packet = bytes([0xAA]) + data_bytes + bytes([checksum, 0x55])

            self.serial.write(packet)
            self.get_logger().info(f"Sent to ESP: {packet.hex()}")

        except Exception as e:
            self.get_logger().error(f"Error sending to ESP: {e}")

    # ------------------------------
    # NEW: Subscriber callback → receives commands from ROS and sends to ESP32
    # ------------------------------
    def command_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)

            voltage = float(payload.get("voltage", 0))
            current = float(payload.get("current", 0))
            ampHours = float(payload.get("ampHours", 0))

            self.send_to_esp(voltage, current, ampHours)

        except Exception as e:
            self.get_logger().error(f"Invalid command payload: {msg.data} | {e}")


def main(args=None):
    rclpy.init(args=args)
    battery_data_publisher = BatteryDataPublisher()

    try:
        rclpy.spin(battery_data_publisher)
    except KeyboardInterrupt:
        print("accepted keyboard interupt")
    finally:
        battery_data_publisher.on_cleanup()
        battery_data_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
