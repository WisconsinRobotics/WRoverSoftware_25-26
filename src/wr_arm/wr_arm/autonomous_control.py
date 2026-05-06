import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64
#from custom_msgs_srvs.msg import GripperPosition
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool
import math

MODIFIER_X = (175/100) #Convert counts to mm [there are 126.4 mm in 100 counts of x_movement]
MODIFIER_Y = (135.6/100) #Convert counts to mm [there are 135.6 mm in 100 counts of y_movement]
OFFSET_Y = 215 #We start -200 mm offset from center
OFFSET_X = 74
WRIST_SPEED_VALUE = .5
GRIPPER_SPEED_VALUE = .5
IN_OUT_MOVE = 40 /1.5
CLOSE_ENOUGH = 1

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
        self.subscription_buttons = self.create_subscription(
            Float64MultiArray,
            'keyboard_center',
            self.listener_callback_keyboard_center,
            10)
        
        #Current limiting the linear rail control
        self.current_side_to_side = self.create_subscription(
            Bool,
            'current_side_to_side',
            self.arm_listener_side_to_side_current,
            10)
        self.side_to_side_can_go = True
        self.msg_side_to_side_zero = Float64()
        self.msg_side_to_side_zero.data = 0.0
        
        self.current_up_and_down = self.create_subscription(
            Bool,
            'current_side_to_side',
            self.arm_listener_up_and_down_current,
            10)
        self.up_and_down_can_go = True
        self.msg_up_and_down_zero = Float64()
        self.msg_up_and_down_zero.data = 0.0

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
        self.D_PAD = [0,0,0,0,0,0,0,0] #Array to keep track of which buttons are pressed
 
        #Set autonmous flag
        self.autonomous = False

        #Set current limit flags
        self.moving_right = False
        self.moving_left = False
        self.hit_left = False
        self.hit_right = False


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
        self.msg_gripper.data = 0.0
        
        #Autonomous messages
        self.going_down = 0
        self.modifier = 1
        self.counter_x = 0
        self.counter_y = 0
        self.counter_in_out = 0
        self.key_counter = 0
        self.x_done = False
        self.y_done = False
        self.centered = False
        self.key_position = []
        self.start_moving_to_actual_center = False

        #Autonomous timer
        
        self.move_to_center_timer = self.create_timer(0.1, self.move_to_actual_center)
        self.timer_autonomous = self.create_timer(0.1, self.autonomous_movement)

    
    #Put publishers in timer to limit rate of publishing
    def timer_callback(self):
        #self.get_logger().info("Hit left: " + str(self.hit_left) + " Hit right: " + str(self.hit_right))
        if(self.hit_right or self.hit_left):
            self.arm_publisher_side_to_side.publish(self.msg_side_to_side_zero)
        else:
            self.arm_publisher_side_to_side.publish(self.msg_side_to_side)

        if(self.up_and_down_can_go):
            self.arm_publisher_up_and_down.publish(self.msg_up_and_down)
        else:
            self.arm_publisher_up_and_down.publish(self.msg_up_and_down_zero)

        self.arm_publisher_forwards_and_bacwards.publish(self.msg_forwards_and_backwards)
        self.arm_publisher_wrist_left.publish(self.msg_wrist_left)
        self.arm_publisher_wrist_right.publish(self.msg_wrist_right)
        self.arm_publisher_gripper.publish(self.msg_gripper)


    def get_linear_rail_speed(self, left, right) -> Float64:
        #Reverse if right is positive
        #Converting -1 -> 1 range of triggers to 0->1
        return (-(left+1)/2 + (right+1)/2)

    
    def timer_update_wrist(self):
        #Publishing
        self.set_wrist_speeds(self.D_PAD[0],self.D_PAD[1],self.D_PAD[2],self.D_PAD[3], self.D_PAD[4],self.D_PAD[5])


    def set_wrist_speeds(self, up, down, left, right, x, y) -> Float32MultiArray:
        if(x==1):
            self.modifier = 3
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
            self.msg_wrist_right.data = (WRIST_SPEED_VALUE)*self.modifier
            self.msg_wrist_left.data = -(WRIST_SPEED_VALUE)*self.modifier
            self.going_down = 0
        elif left == 1:
            self.msg_wrist_right.data = -(WRIST_SPEED_VALUE)*self.modifier
            self.msg_wrist_left.data = (WRIST_SPEED_VALUE)*self.modifier
            self.going_down = 0
        else:
            # if(self.going_down>0):
            #     self.msg_wrist_right.data = WRIST_SPEED_VALUE*self.modifier
            #     self.msg_wrist_left.data = WRIST_SPEED_VALUE*self.modifier
            #     self.going_down -= 1
            # else:
                self.msg_wrist_right.data = 0.0
                self.msg_wrist_left.data = 0.0 
    

    #Read check current to see if we have limit
    def arm_listener_side_to_side_current(self, msg):
        
        if(self.moving_right and not  msg.data):
            self.hit_right = True
        if(self.moving_left and not msg.data):
            self.hit_left = True
        if(self.hit_right and self.moving_left):
            self.hit_right = False
        if(self.hit_left and self.moving_right):
            self.hit_left = False

    #Read check current to see if we have limit
    def arm_listener_up_and_down_current(self, msg):
        self.up_and_down_can_go = msg.data
    
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
        if(self.D_PAD[7] == 1):
            self.autonomous = False
        if(self.D_PAD[7] == 1 and self.D_PAD[6] == 1):
            self.autonomous = True
        if(self.autonomous == False):
            #Expecting (left trigger, rigt trigger)
            linear_rail_speed = self.get_linear_rail_speed(motion[3], motion[2])
            
            if(linear_rail_speed > 0):
                self.moving_left = True
                self.moving_right = False
            elif(linear_rail_speed < 0):
                self.moving_left = False
                self.moving_right = True
            else:
                self.moving_left = False
                self.moving_right = False
            
            #self.get_logger().info("Moving left: " + str(self.moving_left) + " Moving right: " + str(self.moving_right))
            #Expecting (left y joystick)
            self.msg_up_and_down.data = motion[0]

            #Expecting (right y joystick)
            self.msg_forwards_and_backwards.data = -motion[1]  

            #Publishing
            self.msg_side_to_side.data = linear_rail_speed

    def listener_callback_buttons(self, msg):
        buttons = msg.data
        
        #Expecting D-Pad
        self.D_PAD = [buttons[0], buttons[1], buttons[2], buttons[3], buttons[6],buttons[7], buttons[8],buttons[9]]
         # up, down, left, right, x, y, Bumper Left, Bumper Right
        
        #self.get_logger().info('I heard: "%s"' % self.D_PAD)
        #Expecting A and B buttons
        gripper_speed = self.get_gripper_speed(buttons[4], buttons[5])

        self.msg_gripper.data = float(gripper_speed)


    def move_to_actual_center(self):
        
        if(self.start_moving_to_actual_center):
            if(self.counter_x*MODIFIER_X < abs(OFFSET_X)):
                self.counter_x +=1
                if(OFFSET_X < 0):
                    self.msg_side_to_side.data = 1.0
                else:
                    self.msg_side_to_side.data = -1.0
            else:
                self.msg_side_to_side.data = 0.0

            if(self.counter_y*MODIFIER_Y < abs(OFFSET_Y)):
                self.counter_y +=1
                if(OFFSET_Y < 0):
                    self.msg_up_and_down.data = 1.0
                else:
                    self.msg_up_and_down.data = -1.0
            else:
                self.msg_up_and_down.data = 0.0
            
            self.get_logger().info("side_to_side: " + str(self.msg_side_to_side.data) + " up_down: " + str(self.msg_up_and_down.data) + " counter x: " + str(self.counter_x) + " counter y: " + str(self.counter_y) )

            #Only actiavet if bumper left and right are held to prevent accidental activation
            if(self.D_PAD[6] == 1): #If Y button is held, stay centered
                self.counter_y = 0
                self.counter_x = 0
                self.msg_side_to_side.data = 0.0
                self.msg_up_and_down.data = 0.0
                self.centered = True
                self.start_moving_to_actual_center = False

    def listener_callback_keyboard_center(self, msg):
        #msg should be [x center,  y center] of target
        # left of screen is negative x
        # top of screen positive y
        x = msg.data[0]
        y = msg.data[1]
        
        
        #Set to true if autonomous runs
        self.autonomous = True
        self.get_logger().info("AUTO: " + str(self.autonomous))
        self.get_logger().info("centered: " + str(self.centered))
        self.get_logger().info("start_moving_to_actual_center: " + str(self.start_moving_to_actual_center))
        if(self.autonomous == True and self.centered == False and self.start_moving_to_actual_center == False):
            if(abs(x) < CLOSE_ENOUGH and abs(y) < CLOSE_ENOUGH):
                self.start_moving_to_actual_center = True
            else:
                if(x < 0):
                    self.msg_side_to_side.data = min(1.0, abs(x/5))
                else:
                    self.msg_side_to_side.data = max(-1.0, -abs(x/5))
                if(y < 0):
                    self.msg_up_and_down.data = min(1.0, abs(y/5))
                else:
                    self.msg_up_and_down.data = max(-1.0, -abs(y/5))
                if(abs(x) < CLOSE_ENOUGH):
                    self.msg_side_to_side.data = 0.0
                if(abs(y) < CLOSE_ENOUGH):
                    self.msg_up_and_down.data = 0.0
            self.get_logger().info("x: " + str(x) + " y: " + str(y) + " centered: " + str(self.centered))

    def listener_callback_key_positions(self, msg):
        #msg should be [x away,  y away] from target
        # left of screen is negative x
        # top of screen positive y
        # center is 0,0

        self.key_position = msg.data

    def autonomous_movement(self):


        self.get_logger().info("Key Positions: " + str(self.key_position))
        self.get_logger().info("CENTERED: " + str(self.centered))
        
        if 2*self.key_counter + 1 >= len(self.key_position):
            #TODO: set end condition
            if(len(self.key_position)>0):
                self.get_logger().info("Completed all" + str(len(self.key_position)/2) + " keys, stopping autonomous movement")
            return
        
        if(self.centered == True and self.autonomous == True):
            if(self.key_counter < 1):
                x = self.key_position[0 + 2*self.key_counter]
                y = self.key_position[1 + 2*self.key_counter]
            else:
                x = self.key_position[0 + 2*self.key_counter] - self.key_position[0 + 2*(self.key_counter-1)] 
                y = self.key_position[1 + 2*self.key_counter] - self.key_position[1 + 2*(self.key_counter-1)] 
            self.get_logger().info("Current Key Positions from position: " + str(x) + "  " + str(y))

            #move x:
  
            if(self.counter_x*MODIFIER_X < abs(x) and self.x_done == False): #TODO, scale this properly
                self.counter_x +=1
                if(x < 0):
                    self.msg_side_to_side.data = 1.0
                else:
                    self.msg_side_to_side.data = -1.0
            else:
                self.msg_side_to_side.data = 0.0
                #if(self.D_PAD[6] == 1 ): #Prevent  saying x is done until we hold the bumper 
                self.x_done = True
            #move y:
            if(self.counter_y*MODIFIER_Y < abs(y) and self.y_done == False): #TODO, scale this properly
                self.counter_y +=1
                if(y < 0):
                    self.msg_up_and_down.data = 1.0
                else:
                    self.msg_up_and_down.data = -1.0
            else:
                self.msg_up_and_down.data = 0.0
                #if(self.D_PAD[6] == 1): #Prevent  saying x is done until we hold the bumper 
                self.y_done = True
        
        if(self.y_done and self.x_done):
            self.counter_in_out += 1
            self.get_logger().info("Moving in and OUT with counter " + str(self.counter_in_out))
            if(self.counter_in_out < IN_OUT_MOVE):
                self.msg_forwards_and_backwards.data = 1.0
            elif(self.counter_in_out >= IN_OUT_MOVE and self.counter_in_out < 2 * IN_OUT_MOVE):
                self.msg_forwards_and_backwards.data = -1.0
            elif(self.counter_in_out > 2*IN_OUT_MOVE):
                self.counter_in_out = 0.0
                self.msg_forwards_and_backwards.data = 0.0
                self.msg_up_and_down.data = 0.0
                self.counter_x = 0
                self.counter_y = 0
                self.y_done = False
                self.x_done = False
                #TODO: go to next keyboard
                self.key_counter += 1
                self.get_logger().info("Completed key " + str(self.key_counter))

    


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

