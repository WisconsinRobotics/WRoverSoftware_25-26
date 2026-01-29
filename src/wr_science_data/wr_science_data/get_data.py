import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, String, MultiArrayDimension

import serial


class SensorsRawNode(Node):
    def __init__(self):
        super().__init__('sensors_raw')

        # Publishers
        self.pub_fluoro = self.create_publisher(Int16MultiArray, '/sci_fluoro_raw', 10)
        self.pub_soil = self.create_publisher(Int16MultiArray, '/sci_soil_raw', 10)
        self.pub_arduino = self.create_publisher(String, '/sci_arduino_messages', 10)

        # Parameters
        self.serial_data_port = self.declare_parameter(
            'serial_data_port', '/dev/ttyUSB0'
        ).value
        self.serial_baud_rate = self.declare_parameter(
            'serial_baud_rate', 9600
        ).value

        # Subscription
        self.subscription_buttons = self.create_subscription(
            Int16MultiArray,
            'buttons_arm',
            self.listener_callback_buttons,
            10
        )

        # Serial setup
        try:
            self.ser = serial.Serial(
                port=self.serial_data_port,
                baudrate=self.serial_baud_rate,
                timeout=0.5
            )
        except Exception as e:
            self.get_logger().error(f"Serial connection failed: {e}")
            rclpy.shutdown()
            return

        # --------- Fluorometer message setup ----------
        self.fluoro_vals = Int16MultiArray()

        dim1 = MultiArrayDimension()
        dim1.label = "rows"
        dim1.size = 2
        dim1.stride = 18

        dim2 = MultiArrayDimension()
        dim2.label = "cols"
        dim2.size = 9
        dim2.stride = 9

        self.fluoro_vals.layout.dim = [dim1, dim2]

        # Pre-fill wavelengths + empty readings
        wavelengths = [415, 445, 480, 515, 555, 590, 630, 680, 0]
        readings = [0] * 9

        self.fluoro_vals.data = wavelengths + readings

        # --------- Soil message setup ----------
        self.soil_vals = Int16MultiArray()

        soil_dim = MultiArrayDimension()
        soil_dim.label = "soil"
        soil_dim.size = 2
        soil_dim.stride = 2

        self.soil_vals.layout.dim = [soil_dim]
        self.soil_vals.data = [0, 0]

        # Clear serial buffer
        self.ser.reset_input_buffer()

        self.get_logger().info("Finished init")
        self.open = False

        # Timer
        #self.timer = self.create_timer(0.05, self.operate)

    def operate(self):
        try:
            line = self.ser.readline().decode(errors='ignore').strip()

            if not line:
                return

            vals = line.split(",")

            # Debug message (no colons)
            if ":" not in vals[0]:
                msg = String()
                msg.data = line
                self.pub_arduino.publish(msg)
                return

            # Fluorometer (9 values)
            if len(vals) == 9:
                readings = [int(val.split(":")[1]) for val in vals]
                self.fluoro_vals.data[9:] = readings
                self.pub_fluoro.publish(self.fluoro_vals)

            # Soil sensor (2 values)
            elif len(vals) == 2:
                self.soil_vals.data = [
                    int(val.split(":")[1]) for val in vals
                ]
                self.pub_soil.publish(self.soil_vals)

        except Exception as e:
            self.get_logger().warn(f"Serial parse error: {e}")

    def control_arduino(self, a, b, x, y):
        if y == 1:
            self.ser.write(b"s")
            self.get_logger().info("Turning pump")

        if (x == 1 and self.open == False):
            self.get_logger().info("Opening chute")
            self.ser.write(b"o")
            self.open = True
        elif (x == 1 and self.open):
            self.get_logger().info("Closing chute")
            self.ser.write(b"c")
            self.open = False

    def listener_callback_buttons(self, msg):
        buttons = msg.data
        self.get_logger().info("buttons: " + str(buttons))
        if len(buttons) >= 8:
            self.control_arduino(
                buttons[4],
                buttons[5],
                buttons[6],
                buttons[7]
            )


def main(args=None):
    rclpy.init(args=args)

    node = SensorsRawNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()