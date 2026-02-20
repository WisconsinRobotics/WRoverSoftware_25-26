import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64
import math

class SwerveControlSubsrciber(Node):

    def __init__(self):
        super().__init__('swerve_control')
        #TODO:CHANGE IDS
        self.vesc_ids = {"CAROUSEL":["79"],
                         "AUGER":["80"],
                         "INSERT":["78"]
                        }
        self.max_rpm = 6000
        self.limit_rotation = 0

        self.subscription_carousel = self.create_subscription(
            Float64,
            'caroussel',
            self.listener_carousel,
            10)

        self.subscription_auger = self.create_subscription(
            Float64,
            'drill_on_off',
            self.listener_auger,
            10)
        
        self.subscription_insert = self.create_subscription(
            Float64,
            'drill_up_down',
            self.listener_insert,
            10)
                

        self.publisher_ = self.create_publisher(String, 'can_msg', 10)


    def listener_carousel(self, msg):
        can_msg_rpm = String()

        rpm = msg.data * self.max_rpm * 1.5
        can_msg_rpm.data = self.vesc_ids["CAROUSEL"][0] + " CAN_PACKET_SET_RPM " + str(rpm) + " float"
        self.publisher_.publish(can_msg_rpm)


    def listener_auger(self, msg):
        can_msg_duty = String()

        duty = msg.data
        can_msg_duty.data = self.vesc_ids["AUGER"][0] + " CAN_PACKET_SET_DUTY " + str(duty) + " float"
        self.publisher_.publish(can_msg_duty)


    def listener_insert(self, msg):
        can_msg_duty = String()

        duty = msg.data
        can_msg_duty.data = self.vesc_ids["INSERT"][0] + " CAN_PACKET_SET_DUTY " + str(duty) + " float"
        self.publisher_.publish(can_msg_duty)


    def listener_chute(self, msg):
        can_msg_servo = String()

        # I cannot find any CAN command for controlling a servo so it will not be implemented
        #servo = msg.data
        #can_msg_servo.data = self.vesc_ids["INSERT"][0] + " CAN_PACKET_SET_SERVO " + str(duty) + " float"
        #self.publisher_.publish(can_msg_servo)

def main(args=None):
    rclpy.init(args=args)

    swerve_control_subscriber = SwerveControlSubsrciber()
    print("Starting ros2 arm_neo node")
    rclpy.spin(swerve_control_subscriber)
    

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    swerve_control_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
