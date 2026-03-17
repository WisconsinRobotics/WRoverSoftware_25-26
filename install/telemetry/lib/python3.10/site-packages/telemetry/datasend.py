import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

class EspCommander(Node):
    def __init__(self):
        super().__init__('esp_commander')

        # Publisher to the existing topic
        self.pub = self.create_publisher(String, 'esp_commands', 10)
        self.get_logger().info("ESP Commander node ready")

    def send(self, voltage, current, ampHours):
        msg = String()
        payload = {
            "voltage": voltage,
            "current": current,
            "ampHours": ampHours
        }
        msg.data = json.dumps(payload)

        self.pub.publish(msg)
        self.get_logger().info(f"Sent: {msg.data}")

def main(args=None):
    rclpy.init(args=args)

    node = EspCommander()

    # EXAMPLE — modify these values freely
    node.send(50.0, 0.8, 50.0)
    time.sleep(0.1)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
