import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64
#from custom_msgs_srvs.msg import GripperPosition
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Float64MultiArray
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
        self.subscription_buttons = self.create_subscription(
            Float64MultiArray,
            'keyboard_key_positions',
            self.listener_callback_key_positions,
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
        self.kohler_shift = 135.0
        self.D_PAD = [0,0,0,0,0,0] #Array to keep track of which buttons are pressed
        self.absolute_wrist = self.kohler_shift#Start at zero
        self.wrist_positions = [0.0 + self.absolute_wrist ,0.0 + self.absolute_wrist] #[lef,right]
 
        #Set autonmous flag
        self.autonomous = False

        #Define messages beforehand
        self.msg_side_to_side = Float64()
        self.msg_side_to_side.data = 0.0

        self.msg_up_and_down = Float64()
        self.msg_up_and_down.data = 0.0

        self.msg_forwards_and_backwards = Float64()
        self.msg_forwards_and_backwards.data = 0.0

        self.msg_wrist_left = Float64()
        self.msg_wrist_left.data = self.absolute_wrist 

        self.msg_wrist_right = Float64()
        self.msg_wrist_right.data = self.absolute_wrist 

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

    
    def timer_update_wrist(self):
        #Publishing
        
        if self.absolute_wrist >= 0 + self.kohler_shift and self.absolute_wrist <= 100 + self.kohler_shift and 1 in self.D_PAD:
            #self.get_logger().info(str(self.absolute_wrist))
            self.get_wrist_position(self.D_PAD[0],self.D_PAD[1],self.D_PAD[2],self.D_PAD[3], self.D_PAD[4], self.D_PAD[5])
            self.msg_wrist_right.data = float(self.wrist_positions[0])
            self.msg_wrist_left.data = float(self.wrist_positions[1])


    def get_wrist_position(self, up, down, left, right,x,y) -> Float32MultiArray:
        if(x==1):
            self.modifier = 1
        elif(y==1):
            self.modifier = 1
        else:
            self.modifier = 1
        if up == 1:
            self.wrist_positions[0] += WRIST_SPEED_VALUE*self.modifier
            self.wrist_positions[1] += WRIST_SPEED_VALUE*self.modifier
            #if self.absolute_wrist >= 0 + self.kohler_shift + 1:
            #    self.absolute_wrist += -WRIST_SPEED_VALUE #COMMENTED OUT LIMITS, can zero from bottom
        elif down == 1:
            self.wrist_positions[0] += -WRIST_SPEED_VALUE*self.modifier
            self.wrist_positions[1] += -WRIST_SPEED_VALUE*self.modifier
            #if self.absolute_wrist <= 100 + self.kohler_shift - 1:
            #    self.absolute_wrist += WRIST_SPEED_VALUE
        elif left == 1:
            self.wrist_positions[0] += WRIST_SPEED_VALUE*self.modifier
            self.wrist_positions[1] += -WRIST_SPEED_VALUE*self.modifier
        elif right == 1:
            self.wrist_positions[0] += -WRIST_SPEED_VALUE*self.modifier
            self.wrist_positions[1] += WRIST_SPEED_VALUE*self.modifier
    
     
    
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

        #Turn off autonmous by holding down left and right trigger #TODO check that 1 is actually published when you hold down triggers
        if(motion[2] >= 0.9 and motion[3] >= 0.9):
            self.autonomous = False
        if(self.autonomous == False):
            #Expecting (left trigger, rigt trigger)
            linear_rail_speed = self.get_linear_rail_speed(motion[3], motion[2])

            #Expecting (left y joystick)
            self.msg_up_and_down.data = motion[0]

            #Expecting (right y joystick)
            self.msg_forwards_and_backwards.data = -motion[1]  

            #Publishing
            self.msg_side_to_side.data = linear_rail_speed

    def listener_callback_buttons(self, msg):
        buttons = msg.data
        
        #Expecting D-Pad
        self.D_PAD = [buttons[0], buttons[1], buttons[2], buttons[3], buttons[6],buttons[7]] # up, down, left, right, x, y
        
        #Expecting A and B buttons
        gripper_speed = self.get_gripper_speed(buttons[4], buttons[5])
        

        self.msg_gripper.data = float(gripper_speed)
    def listener_callback_key_positions(self, msg):
        #msg should be [x away,  y away] from target
        # left of screen is negative x
        # top of screen positive y
        x = msg.data[0]
        y = msg.data[1]
        close_enough = 10
        
        #Set to true if autonomous runs
        self.autonomous = True
        if(x < close_enough and y < close_enough):
            self.msg_forwards_and_backwards.data = 1
        else:
            self.msg_forwards_and_backwards.data = -1

            #Publishing
            if(x < 0):
                self.msg_side_to_side.data = 1
            else:
                self.msg_side_to_side.data = -1
            if(y < 0):
                self.msg_up_and_down.data = 1
            else:
                self.msg_up_and_down.data = -1


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

