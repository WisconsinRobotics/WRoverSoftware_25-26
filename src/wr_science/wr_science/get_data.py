import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import String

import numpy as np
import serial
import traceback 

class SensorsRawNode(Node):
    def __init__(self):
        super().__init__('sensors_raw')
        self.pub_fluoro = self.create_publisher(Int16MultiArray, '/sci_fluoro_raw', 1)
        self.pub_soil = self.create_publisher(Int16MultiArray, '/sci_soil_raw', 1)
        self.pub_arduino = self.create_publisher(String, '/sci_arduino_messages', 1)
        
        self.serial_data_port = self.declare_parameter('/serial_data_port', '/dev/ttyUSB0').value
        self.serial_baud_rate = self.declare_parameter('/serial_baud_rate', 9600).value
        
        
        self.subscription_buttons = self.create_subscription(
            Int16MultiArray,
            'buttons_arm',
            self.listener_callback_buttons,
            10)
        # initiate the serial connection with the arduino
        try:
            self.ser = serial.Serial(
                port=self.serial_data_port,
                baudrate=self.serial_baud_rate,
                timeout=0.5
            )
        except Exception as e:
            self.get_logger().error(
                f"USB connection to Arduino unsuccessful: {e}"
            )
            raise

        # # initialize empty fluorometer message
        # self.fluoro_vals = Int16MultiArray()
        # #self.fluoro_vals.layout.label[0] = "colors"
        # self.fluoro_vals.layout.size[0] = 9
        # #self.fluoro_vals.layout.label[1] = "wavelength, reading"
        # self.fluoro_vals.layout.size[1] = 2
        # self.fluoro_vals.data = np.array([[415,445,480,515,555,590,630,680,0],
        #                                   [0,  0,  0,  0,  0,  0,  0,  0,  0]])

        # # initialize empty soil sensor message
        # self.soil_vals = Int16MultiArray()
        # #self.soil_vals.layout.label[0] = "measurements"
        # self.soil_vals.layout.size[0] = 2
        # self.soil_vals.data = [0,0]

        # # Flush any backlog and read a line to make sure the port input is current. 
        # self.ser.flushInput()
        # self.ser.readline()

        self.green_value = 0
        self.orange_value = 0
        self.methane_value = 0.0

        with open("sensor_data.csv", "w") as f:
            f.write("Green,Orange,Methane\n")

        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.operate)

     

        
    def operate(self):

        try:
            line = self.ser.readline().decode(errors='ignore').strip()

            if not line:
                return

            print(line)

            # Store latest values
            if "Green (515nm):" in line:
                self.green_value = int(line.split(":")[1].strip())

            elif "Orange (590nm):" in line:
                self.orange_value = int(line.split(":")[1].strip())

            elif "Methane sensor Voltage:" in line:
                self.methane_value = float(line.split(":")[1].strip())

                # Once methane arrives, save full row to CSV
                with open("sensor_data.csv", "a") as f:
                    f.write(f"{self.green_value},{self.orange_value},{self.methane_value}\n")

                self.get_logger().info(
                    f"Saved: {self.green_value}, "
                    f"{self.orange_value}, "
                    f"{self.methane_value}"
                )

        except Exception as e:
            self.get_logger().error(f"Parse error: {e}")

    def control_arduino(self, a, b, x, y, D_PAD_UP):
        if(y==1):
            self.ser.write("o".encode()) #Open
            self.get_logger().info("OPEN")
        if(x==1):
            self.ser.write("c".encode()) #Close
            self.get_logger().info("CLOSE")
        if(D_PAD_UP==1):
            self.ser.write("p".encode()) #Turn on pump when releasing
       
    def listener_callback_buttons(self, msg):
        buttons = msg.data
    
        self.control_arduino(buttons[4], buttons[5], buttons[6], buttons[7], buttons[0])

def main(args=None):
    try: 
        rclpy.init(args=args)
        print('sensors_raw beginning')
        sensors_raw_instance = SensorsRawNode()
        rclpy.spin(sensors_raw_instance)
    except KeyboardInterrupt:
        print('Keyboard Interrupt')
        
    sensors_raw_instance.destroy_node()
    rclpy.shutdown()
    
    print('sensors_raw exiting')


if __name__ == '__main__':
    main()

