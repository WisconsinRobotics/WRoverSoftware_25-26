import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64
#from custom_msgs_srvs.msg import GripperPosition
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int16MultiArray
import math


WRIST_SPEED_VALUE = .5
GRIPPER_SPEED_VALUE = .5
class ArmLogic(Node):

    def __init__(self):
        super().__init__('arm_logic_vel')
        self.subscription_joy = self.create_subscription(
            Float32MultiArray,
            'joy_arm',
            self.listener_callback_joy,
            10)
        
        self.subscription_buttons = self.create_subscription(
            Int16MultiArray,
            'buttons_arm',
            self.listener_callback_buttons,
            10)

        self.pub_drill_on_off = self.create_publisher(Float64, 'drill_on_off', 10)
        self.pub_drill_up_down = self.create_publisher(Float64, 'drill_up_down', 10)
        
        self.pub_caroussel = self.create_publisher(Float64, 'caroussel', 10)

        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        #Define Postion of left and right position of wrist
        self.kohler_shift = 130
        self.D_PAD = [0,0,0,0,0,0] #Array to keep track of which buttons are pressed
        #self.absolute_wrist = 50 + self.kohler_shift#Start at zero
        #self.wrist_positions = [0.0 + self.absolute_wrist ,0.0 + self.absolute_wrist] #[lef,right]
 

        #Define messages beforehand
        self.msg_drill_on_off = Float64()
        self.msg_drill_on_off.data = 0.0

        self.msg_up_and_down = Float64()
        self.msg_up_and_down.data = 0.0

        self.msg_caroussel = Float64()
        self.msg_caroussel.data = 0.0

    
    #Put publishers in timer to limit rate of publishing
    def timer_callback(self):
        self.pub_drill_on_off.publish(self.msg_drill_on_off)
        self.pub_drill_up_down.publish(self.msg_up_and_down)
        self.pub_caroussel.publish(self.msg_caroussel)
      

    def get_linear_rail_speed(self, left, right) -> Float64:
        #Reverse if right is positive
        #Converting -1 -> 1 range of triggers to 0->1
        return ((left+1)/2 - (right+1)/2)
    
    # def set_wrist_speeds(self, up, down, left, right) -> Float32MultiArray:
    #     #Assume left is forward
    #     wrist_speeds = [0,0]
    #     if up == 1:
    #         wrist_speeds = [WRIST_SPEED_VALUE, -WRIST_SPEED_VALUE]
    #         return wrist_speeds
    #     elif down == 1:
    #         wrist_speeds = [-WRIST_SPEED_VALUE, WRIST_SPEED_VALUE]
    #         return wrist_speeds
    #     elif left == 1:
    #         wrist_speeds = [WRIST_SPEED_VALUE, WRIST_SPEED_VALUE]
    #         return wrist_speeds
    #     elif right == 1:
    #         wrist_speeds = [-WRIST_SPEED_VALUE, -WRIST_SPEED_VALUE]
    #         return wrist_speeds
    #     else:
    #         return wrist_speeds
    

    
    def get_gripper_speed(self, a, b) -> float:
        if a == 1:
            return GRIPPER_SPEED_VALUE
        elif b == 1:
            return -GRIPPER_SPEED_VALUE
        else:
            return 0

    def listener_callback_joy(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg.data)
        motion = msg.data

        #Expecting (left y joystick)
        self.msg_up_and_down.data = motion[0]

        #Expecting (left trigger, rigt trigger)
        linear_rail_speed = self.get_linear_rail_speed(motion[3], motion[2])

        #Publishing
        self.msg_caroussel.data = linear_rail_speed

    def listener_callback_buttons(self, msg):
        buttons = msg.data
        
        #Expecting A and B buttons
        gripper_speed = self.get_gripper_speed(buttons[4], buttons[5])
        
        self.msg_drill_on_off.data = float(gripper_speed)
    


def main(args=None):
    rclpy.init(args=args)
    swerve_subscriber = ArmLogic()
    print("Starting ros2 arm_logic node")
    rclpy.spin(swerve_subscriber)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    swerve_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

