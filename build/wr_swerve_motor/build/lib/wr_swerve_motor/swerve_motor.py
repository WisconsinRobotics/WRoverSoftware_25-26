import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
import math

def get_wheel_vectors(vehicle_translation, rotational_velocity):
    # x component is vehicle_translation +/- rot_vel*body_width/2
    # y component is vehicle_translation +/- rot_vel*body_height/2
    # + or - depends on what wheel it is


    # Kohler added code here to handle triggers

    rotational_velocity = -((float(rotational_velocity[0]) + 1) / 2.0) + ((float(rotational_velocity[1]) + 1.0) / 2.0)

    #print("Rotational velocity", rotational_velocity)

    #TODO GET BODY HEIGHT AND WIDTH
    BODY_HEIGHT = 0.93
    BODY_WIDTH = 0.60
    # see "derivation of inverse kinematics for swerve" figure 5
    # possible x components    
    A = vehicle_translation[0] - ((rotational_velocity)*(BODY_HEIGHT/2))
    B = vehicle_translation[0] + ((rotational_velocity)*(BODY_HEIGHT/2))
    # possible y components
    C = vehicle_translation[1] - ((rotational_velocity)*(BODY_WIDTH/2))
    D = vehicle_translation[1] + ((rotational_velocity)*(BODY_WIDTH/2))
    FL_vector = [B, D]
    FR_vector = [B, C]
    BL_vector = [A, D]
    BR_vector = [A, C]

    return [FL_vector, FR_vector, BL_vector, BR_vector]

def get_wheel_speed(wheel_vector):
    return math.sqrt(wheel_vector[0]*wheel_vector[0] + wheel_vector[1]*wheel_vector[1])

def get_wheel_speeds(wheel_vectors):
# returns a list containing vectors [front left, front right, back left, back right]
    return [get_wheel_speed(wheel_vectors[0]),
            get_wheel_speed(wheel_vectors[1]),
            get_wheel_speed(wheel_vectors[2]),
            get_wheel_speed(wheel_vectors[3])]

def get_wheel_angle(wheel_vector):
    # arctan2(x_component, y_component)
    return (math.atan2((wheel_vector[0]), wheel_vector[1])) * (180/math.pi)

def get_wheel_angles(wheel_vectors):
    return [get_wheel_angle(wheel_vectors[0]),
            get_wheel_angle(wheel_vectors[1]),
            get_wheel_angle(wheel_vectors[2]),
            get_wheel_angle(wheel_vectors[3])]

class SwerveSubscriber(Node):

    def __init__(self):
        super().__init__('swerve_motor')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'swerve',
            self.listener_callback,
            10)

        self.swerve_publisher_FL = self.create_publisher(Float32MultiArray, 'swerve_FL', 10)
        self.swerve_publisher_FR = self.create_publisher(Float32MultiArray, 'swerve_FR', 10)
        self.swerve_publisher_BL = self.create_publisher(Float32MultiArray, 'swerve_BL', 10)
        self.swerve_publisher_BR = self.create_publisher(Float32MultiArray, 'swerve_BR', 10)

        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg.data)
        motion = msg.data
        wheel_vectors = get_wheel_vectors([motion[1],motion[0]],[motion[2], motion[3]])
        wheel_speeds  = get_wheel_speeds(wheel_vectors)
        wheel_angles  = get_wheel_angles(wheel_vectors)
        print("Wheel Speeds", wheel_speeds)
        print("Wheel Angles", wheel_angles)
        for i in range (0, 4):
            if wheel_angles[i] < -90.0:
                wheel_angles[i] += 180.0
                wheel_speeds[i] *= -1.0
            elif wheel_angles[i] > 90.0: 
                wheel_angles[i] -= 180.0
                wheel_speeds[i] *= -1.0


        # TODO: Make this more abstract for actual control
        if (len(wheel_speeds) == 4 and len(wheel_angles) == 4 and
           (wheel_speeds[0] != 0 and wheel_speeds[1] != 0 and
            wheel_speeds[2] != 0 and wheel_speeds[3] != 0)):
            msg_FL = Float32MultiArray()
            msg_FL.data = [wheel_speeds[0],wheel_angles[0]]
            self.swerve_publisher_FL.publish(msg_FL)

            msg_FR = Float32MultiArray()
            msg_FR.data = [wheel_speeds[1],wheel_angles[1]]
            self.swerve_publisher_FR.publish(msg_FR)

            msg_BL = Float32MultiArray()
            msg_BL.data = [wheel_speeds[2],wheel_angles[2]]
            self.swerve_publisher_BL.publish(msg_BL)

            msg_BR = Float32MultiArray()
            msg_BR.data = [wheel_speeds[3],wheel_angles[3]]
            self.swerve_publisher_BR.publish(msg_BR)

            #can_msg_angle = String()
            #can_msg_angle.data = f"74 CAN_PACKET_SET_POS {wheel_speeds[0]} int"

def main(args=None):
    rclpy.init(args=args)
    swerve_subscriber = SwerveSubscriber()
    print("Starting ros2 swerve-motor node")
    rclpy.spin(swerve_subscriber)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    swerve_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


