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

        self.arm_publisher_side_to_side = self.create_publisher(Float64, 'side_to_side', 10)
        self.arm_publisher_up_and_down = self.create_publisher(Float64, 'up_and_down', 10)
        self.arm_publisher_forwards_and_bacwards = self.create_publisher(Float64, 'forwards_and_backwards', 10)
        self.arm_publisher_wrist_left = self.create_publisher(Float64, 'differential_left', 10)
        self.arm_publisher_wrist_right = self.create_publisher(Float64, 'differential_right', 10)
        self.arm_publisher_gripper = self.create_publisher(Float64, 'gripper', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        timer_period = 0.01  # seconds
        self.timer_wrist = self.create_timer(timer_period, self.timer_update_wrist)

        #Define Postion of left and right position of wrist
        self.kohler_shift = 130
        self.D_PAD = [0,0,0,0,0,0] #Array to keep track of which buttons are pressed
        #self.absolute_wrist = 50 + self.kohler_shift#Start at zero
        #self.wrist_positions = [0.0 + self.absolute_wrist ,0.0 + self.absolute_wrist] #[lef,right]
 

        #Define messages beforehand
        self.msg_side_to_side = Float64()
        self.msg_side_to_side.data = 0.0

        self.msg_up_and_down = Float64()
        self.msg_up_and_down.data = 0.0

        self.msg_forwards_and_backwards = Float64()
        self.msg_forwards_and_backwards.data = 0.0

        self.msg_wrist_left = Float64()
        self.msg_wrist_left.data = 0.0

        self.msg_wrist_right = Float64()
        self.msg_wrist_right.data = 0.0

        self.msg_gripper = Float64()
        self.msg_gripper.data = 0.
        
        self.going_down = 0
        self.modifier = 1


    
    #Put publishers in timer to limit rate of publishing
    def timer_callback(self):
        self.arm_publisher_side_to_side.publish(self.msg_side_to_side)
        self.arm_publisher_up_and_down.publish(self.msg_up_and_down)
        self.arm_publisher_forwards_and_bacwards.publish(self.msg_forwards_and_backwards)
        self.arm_publisher_wrist_left.publish(self.msg_wrist_left)
        self.arm_publisher_wrist_right.publish(self.msg_wrist_right)
        self.arm_publisher_gripper.publish(self.msg_gripper)


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
    
    def timer_update_wrist(self):
        #Publishing
        self.set_wrist_speeds(self.D_PAD[0],self.D_PAD[1],self.D_PAD[2],self.D_PAD[3], self.D_PAD[4],self.D_PAD[5])


    def set_wrist_speeds(self, up, down, left, right, x, y) -> Float32MultiArray:
        if(x==1):
            self.modifier = .5
        elif(y==1):
            self.modifier = .3
        else:
            self.modifier = 1
        #self.get_logger().info('I heard: "%s"' % self.modifier)
        if up == 1:
            self.msg_wrist_right.data = WRIST_SPEED_VALUE*self.modifier
            self.msg_wrist_left.data = WRIST_SPEED_VALUE*self.modifier
            self.going_down = 0
        elif down == 1:
            self.msg_wrist_right.data = -WRIST_SPEED_VALUE*self.modifier
            self.msg_wrist_left.data = -WRIST_SPEED_VALUE*self.modifier
            self.going_down = 10
        elif right == 1:
            self.msg_wrist_right.data = -(WRIST_SPEED_VALUE*0.8)*self.modifier
            self.msg_wrist_left.data = (WRIST_SPEED_VALUE*1.2)*self.modifier
            self.going_down = 0
        elif left == 1:
            self.msg_wrist_right.data = (WRIST_SPEED_VALUE*1.2)*self.modifier
            self.msg_wrist_left.data = -(WRIST_SPEED_VALUE*0.8)*self.modifier
            self.going_down = 0
        else:
            if(self.going_down>0):
                self.msg_wrist_right.data = WRIST_SPEED_VALUE*self.modifier
                self.msg_wrist_left.data = WRIST_SPEED_VALUE*self.modifier
                self.going_down -= 1
            else:
                self.msg_wrist_right.data = 0.0
                self.msg_wrist_left.data = 0.0 
 

    
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

        #Expecting (left trigger, rigt trigger)
        linear_rail_speed = self.get_linear_rail_speed(motion[3], motion[2])

        #Expecting (left y joystick)
        self.msg_up_and_down.data = motion[0]

        #Expecting (right y joystick)
        self.msg_forwards_and_backwards.data = motion[1]  

        #Publishing
        self.msg_side_to_side.data = linear_rail_speed

    def listener_callback_buttons(self, msg):
        buttons = msg.data
        
        #Expecting D-Pad
        self.D_PAD = [buttons[0], buttons[1], buttons[2], buttons[3], buttons[6],buttons[7]] # up, down, left, right, x, y
        
        #Expecting A and B buttons
        gripper_speed = self.get_gripper_speed(buttons[4], buttons[5])
        

        self.msg_gripper.data = float(gripper_speed)
    


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

