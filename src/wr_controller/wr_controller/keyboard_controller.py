import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int16MultiArray
import threading
import sys
import select
import termios
import tty

SPEED = 1.0
class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')

        # Publishers
        self.arm_publisher = self.create_publisher(Float32MultiArray, 'joy_arm', 10)
        self.buttons_publisher_ = self.create_publisher(Int16MultiArray, 'buttons_arm', 10)

        # State
        self.motion = [0.0, 0.0, 0.0, 0.0]
        self.buttons = [0] * 6
        self.running = True

        # Start background thread for terminal input
        input_thread = threading.Thread(target=self.keyboard_loop)
        input_thread.daemon = True
        input_thread.start()

        # Timer publishes at fixed rate (for motion updates)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("Keyboard teleop started — hold W/S/Q/A to move, Ctrl-C to quit.")

    def get_key(self):
        """Non-blocking key reader (returns '' if no key pressed)."""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))
        return key

    def keyboard_loop(self):
        """Continuously read keys while running (button-hold behavior)."""
        old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        try:
            while self.running:
                key = self.get_key()

                # Reset motion each loop unless key held
                self.motion = [0.0, 0.0, 0.0, 0.0]
                self.buttons = [0] * 6

                if key == 'w':
                    self.motion[0] = SPEED
                elif key == 's':
                    self.motion[0] = -SPEED
                elif key == 'q':
                    self.motion[1] = SPEED
                elif key == 'a':
                    self.motion[1] = -SPEED
                elif key == 'c':
                    self.get_logger().info("Pressed arm ID (ARM)")
                elif key == 'z':
                    self.buttons[4] = 1
                elif key == 'x':
                    self.buttons[5] = 1
                elif key == 'i':
                    self.buttons[0] = 1
                elif key == 'j':
                    self.buttons[2] = 1
                elif key == 'k':
                    self.buttons[1] = 1
                elif key == 'l':
                    self.buttons[3] = 1
 
                elif key == '\x03':  # Ctrl-C
                    self.running = False
                    break

        except Exception as e:
            self.get_logger().error(f"Keyboard thread error: {e}")
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def timer_callback(self):
        # Publish motion
        swerve_msg = Float32MultiArray()
        swerve_msg.data = self.motion
        self.arm_publisher.publish(swerve_msg)

        # Publish buttons
        buttons_msg = Int16MultiArray()
        buttons_msg.data = self.buttons
        self.buttons_publisher_.publish(buttons_msg)

    def destroy_node(self):
        self.running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard teleop shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

