import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64
import math

BREAK_REL = .5

class ArmControlSubsrciber(Node):

    def __init__(self):
        super().__init__('arm_control')
        self.vesc_ids = {"side_to_side":["80"],
                         "up_and_down":["81"],
                         "forwards_and_backwards":["82"],
                         "differential_left":["83"],
                         "differential_right":["84"],
                         "gripper":["85"],
                        }
        self.max_rpm = 6000
        self.limit_rotation = 0
        self.subscription_side_to_side = self.create_subscription(
            Float64,
            'side_to_side',
            self.arm_listener_side_to_side,
            10)

        self.subscription_FR = self.create_subscription(
            Float64,
            'up_and_down',
            self.arm_listener_up_and_down,
            10)
        
        self.subscription_BL = self.create_subscription(
            Float64,
            'forwards_and_backwards',
            self.arm_listener_forwards_and_backwards,
            10)
                
        self.subscription_BR = self.create_subscription(
            Float64,
            'differential_left',
            self.arm_listener_differential_left,
            10)
        self.subscription_BR = self.create_subscription(
            Float64,
            'differential_right',
            self.arm_listener_differential_right,
            10)
        self.subscription_BR = self.create_subscription(
            Float64,
            'gripper',
            self.arm_listener_gripper,
            10)

        self.publisher_ = self.create_publisher(String, 'can_msg', 10)

 
        self.get_logger().info("Started ARM NODE")

    def arm_listener_side_to_side(self, msg):
        can_msg_rpm = String()
        rpm = msg.data * self.max_rpm * (10/6) #10,000 rpm
        can_msg_rpm.data = self.vesc_ids["side_to_side"][0] + " CAN_PACKET_SET_RPM " + str(rpm) + " float"
        self.publisher_.publish(can_msg_rpm)
        #self.get_logger().info('Publishing RPM FL: "%s"' % can_msg_rpm)

    def arm_listener_up_and_down(self, msg):
        can_msg_rpm = String()
        rpm = msg.data * self.max_rpm* (10/6) #10,000 rpm
        can_msg_rpm.data = self.vesc_ids["up_and_down"][0] + " CAN_PACKET_SET_RPM " + str(rpm) + " float"
        self.publisher_.publish(can_msg_rpm)
        #self.get_logger().info('Publishing RPM FL: "%s"' % can_msg_rpm)


    def arm_listener_forwards_and_backwards(self, msg):
        can_msg_rpm = String()
        rpm = msg.data * self.max_rpm
        can_msg_rpm.data = self.vesc_ids["forwards_and_backwards"][0] + " CAN_PACKET_SET_RPM " + str(rpm) + " float"
        self.publisher_.publish(can_msg_rpm)
        #self.get_logger().info('Publishing RPM FL: "%s"' % can_msg_rpm)


    def arm_listener_differential_left(self, msg):
        can_msg_angle = String()
        speed_amount = msg.data * self.max_rpm 
        if(speed_amount == 0.0):
            can_msg_angle.data = self.vesc_ids["differential_left"][0] + " CAN_PACKET_SET_CURRENT_HANDBRAKE_REL " + str(BREAK_REL) +" float"
        else:
            can_msg_angle.data = self.vesc_ids["differential_left"][0] + " CAN_PACKET_SET_RPM " + str(speed_amount) +" float"
        
        #74 is id; CAN_PACKET_SET_POS is command; turn_amount is angle to turn to divide by 4; float is value to convert to
        self.publisher_.publish(can_msg_angle)
        #self.get_logger().info('Publishing Angle WRIST_LEFT: "%s"' % can_msg_angle)


    def arm_listener_differential_right(self, msg):
        can_msg_angle = String()
        speed_amount = msg.data * self.max_rpm 
        if(speed_amount == 0.0):
            can_msg_angle.data = self.vesc_ids["differential_right"][0] + " CAN_PACKET_SET_CURRENT_HANDBRAKE_REL " + str(BREAK_REL) +" float"
        else:
            can_msg_angle.data = self.vesc_ids["differential_right"][0] + " CAN_PACKET_SET_RPM " + str(speed_amount) +" float"
        
        #74 is id; CAN_PACKET_SET_POS is command; turn_amount is angle to turn to divide by 4; float is value to convert to
        self.publisher_.publish(can_msg_angle)
        #self.get_logger().info('Publishing Angle WRIST_LEFT: "%s"' % can_msg_angle)

    def arm_listener_gripper(self, msg):
        can_msg_rpm = String()
        rpm = msg.data * self.max_rpm
        can_msg_rpm.data = self.vesc_ids["gripper"][0] + " CAN_PACKET_SET_RPM " + str(rpm) + " float"
        self.publisher_.publish(can_msg_rpm)
        #self.get_logger().info('Publishing RPM FL: "%s"' % can_msg_rpm)


def main(args=None):
    rclpy.init(args=args)

    arm_control_subscriber = ArmControlSubsrciber()

    rclpy.spin(arm_control_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    arm_control_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()