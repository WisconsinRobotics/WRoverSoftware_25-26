#!/usr/bin/env python3

import math
import numpy as np
import time
import rclpy
from rclpy.node import Node
import cv_bridge
from std_msgs.msg import Float32MultiArray


'''A dictionary with the 2 circles inside (GNSS points area of search). Each circle contains the number of semicircles
that will be used to search for the aruco tag, each of these with their respective radius, start positions, and end
positions'''
circles = {
    "big_circle" : {
        "semicircle_1" : {
            "radius" : 20,
            "start" : 0,
            "end": lambda theta: math.pi <= theta <= (3 * math.pi) / 2
        },
        "semicircle_2" : {
            "radius" : 17.5,
            "start" : math.pi,
            "end": lambda theta: 0 <= theta <= (math.pi / 2)
        },
        "semicircle_3" : {
            "radius" : 15,
            "start" : 0,
            "end": lambda theta: math.pi <= theta <= (3 * math.pi) / 2
        },
        "semicircle_4" : {
            "radius" : 12.5,
            "start" : math.pi,
            "end": lambda theta: 0 <= theta <= (math.pi / 2)
        },
        "semicircle_5" : {
            "radius" : 10,
            "start" : 0,
            "end": lambda theta: math.pi <= theta <= (3 * math.pi) / 2
        }
    },

    "small_circle" : {
        "semicircle_1" : {
            "radius" : 10,
            "start" : 0,
            "end": lambda theta: math.pi <= theta <= (3 * math.pi) / 2
        },
        "semicircle_2" : {
            "radius" : 7.5,
            "start" : math.pi,
            "end": lambda theta: 0 <= theta <= (math.pi / 2)
        },
        "semicircle_3" : {
            "radius" : 5,
            "start" : 0,
            "end": lambda theta: math.pi <= theta <= (3 * math.pi) / 2
        }
    }
}


def semicircle_segmentation(semicircle, distance):
    '''
    This function divides each semicircle of the search drive into points, which are useful for detecting
    deflections in the trajectory and correcting them. This is also crucial for allowing the obstacle
    avoidance node to correctly run in parallel. This allows it to have a point of reference to go to after
    avoiding an obstacle to continue with the path.
    :param semicircle: the current semicircle the rover is using. See circles (dict) for info.
    :param distance: desired distance between points of reference. (meters).
    :return: 2D list with each point represented as a list.
    '''

    def angle_gaps_between_points(rad, dis):
        '''
        Computes the distance between two points in a circle and uses that two find the angle between those points:

        distance between two points = √((rsinθ)^2 + (rcosθ - r)^2)
        Reducing by trigonometric properties: d = 2rsin(θ/2)

        Solving for theta: θ = 2sin^-1(d/(2r))
        Useful for finding the angle between points with the same distance but in circles with different radius.

        @:param rad: radius of the semicircle the rover is using at that moment.
        @:param distance: the distance between points we want to have
        @:return: Exact angle difference between points in a circle to have the same distance between them all along
        the circumference.
        '''
        theta = 2 * math.asin(dis / (2 * rad))

        return theta


    angle_gap = angle_gaps_between_points(semicircle["radius"], distance)
    # Number of segments for semicircle:
    num_seg = int(math.pi // angle_gap) + 1

    # Number of points to have for the semicircle path:
    num_pts = num_seg + 1

    # We use a rounded version of pi to ease calculations:
    our_pi = round(math.pi, 4)

    # We use a rounded theta to have better-defined angles:
    our_theta = math.floor((our_pi / (math.pi / angle_gap)) * 1000) / 1000

    def create_points_list(theta):
        '''
        Creates a list with each reference point of the current semicircle, which includes the
        x and y position, angle in the semicircle, and rover orientation.
        :param theta: Rounded angle difference that we are going to use (see: angle_gaps_between_points()).
        :return: 2D list with each point represented as a list.
        '''
        pts = []
        make_negative = 0
        if semicircle["start"] != 0:
            make_negative = math.pi
        # Append every reference point of path with respective angle and orientation:
        for i in range (num_seg):
            angle = (i * theta) + make_negative
            # Rover will always turn counter-clockwise and will be perpendicular to the radius:
            orientation = angle + (math.pi / 2)
            if orientation >= 2 * math.pi:
                orientation -= 2 * math.pi
            x_val = semicircle["radius"] * math.cos(angle)
            y_val = semicircle["radius"] * math.sin(angle)
            pts.append({
                "x_position" : x_val,
                "y_position" : y_val,
                "angle" : angle,
                "orientation" : orientation
            })
        # We append the last point with the real value of pi
        if semicircle["start"] == 0:
            pts.append({
                "x_position" : semicircle["radius"] * -1,
                "y_position" : 0.0,
                "angle" : math.pi,
                "orientation" : math.pi + math.pi / 2
            })
        else:
            pts.append({
                "x_position": semicircle["radius"],
                "y_position": 0.0,
                "angle": 0.0,
                "orientation": math.pi / 2
            })
        return pts


    points = create_points_list(our_theta)

    return points


def get_reference_point(current_point, list_of_points, current_semicircle):
    '''
    This function will be run when the rover gets deflected from the established path.
    :param current_point: The current position of the rover. This should be a dictionary.
    :param list_of_points: All the possible points in the current semicircle to which the rover could
    go to reorient itself. This should be a list of dictionaries.
    :param current_semicircle: The current semicircle that the rover should follow. This should be a dictionary with
    radius, start, and end position values
    :return: Returns the point in the semicircle that is closest to the current position of the rover. The angle of the
    polar coordinates of this point has to be greater than the current angle of the polar coordinates of the rover,
    because this way it will keep moving forward. This should be a dictionary.
    '''
    reference_point = 0
    for i in range(len(list_of_points)-1):
        # See if the current angle of the rover is in between two points of the semicircle
        if list_of_points[i]["angle"] <= current_point["angle"] < list_of_points[i + 1]["angle"]:
            # use that point as reference
            reference_point = list_of_points[i+1]

    # Take the first point in the semicircle as reference if the current position of the rover is on the previous
    # quadrant of the start point of the semicircle
    if reference_point == 0:
        if ((current_semicircle["start"] == 0 and (3 * math.pi) / 2 < current_point["angle"] < current_semicircle["start"]) or
                (current_semicircle["start"] != 0 and math.pi / 2 < current_point["angle"] < current_semicircle["start"])):
            reference_point = list_of_points[0]


        else:
            # print the current position_angle
            print("Point of reference not found. Probably already greater than pi or 0.")
            print(f"Current position: {current_point['x_position']}, {current_point['y_position']}, {current_point['angle']}")
            reference_point = list_of_points[-1]

    return reference_point


# def get_current_semicircle(circle, semicircle, semicircle_num, current_info):
#     '''
#     This function updates the semicircle and/or circle values if the rover has already finished the path
#     of a semicircle.
#     :param circle: The current circle the rover is in. Should be a string.
#     :param semicircle: The current semicircle the rover is in. Should be a dictionary with its values.
#     :param semicircle_num: The number of that semicircle. Should be an int.
#     :param current_info: The current info of the rover. Should be a dictionary.
#     :return: Returns the updated (or not) values of the current circle and semicircle.
#     '''
#     # Define the number of semicircled for each circle:
#     if circle == "small":
#         cir_len = 3
#     else:
#         cir_len = 5
#     # Check if the rover has completed the current semicircle:
#     if semicircle["end"](current_info["angle"]):  # Boolean expression from the circles dictionary
#         if semicircle_num < cir_len:
#             semicircle_num += 1
#         else:
#             semicircle_num = 1
#             if circle == "small":
#                 circle = "big"
#             else:
#                 circle = "small"
#
#     return circle, semicircle_num

def get_current_semicircle(circle, semicircle, semicircle_num, current_info, full_search_done):
    '''
    This function updates the semicircle and/or circle values if the rover has already finished the path
    of a semicircle.
    :param circle: The current circle the rover is in. Should be a string.
    :param semicircle: The current semicircle the rover is in. Should be a dictionary with its values.
    :param semicircle_num: The number of that semicircle. Should be an int.
    :param current_info: The current info of the rover. Should be a dictionary.
    :return: Returns the updated (or not) values of the current circle and semicircle.
    '''
    # Define the number of semicircled for each circle:
    cir_len = 3 # 5
    # Check if the rover has completed the current semicircle:
    if semicircle["end"](current_info["angle"]):  # Boolean expression from the circles dictionary
        if semicircle_num < cir_len:
            semicircle_num += 1
        else:
            full_search_done = True

    return circle, semicircle_num, full_search_done


def deflection(current_radius, desired_radius):
    '''
    This function checks if the rover is deflected from th established path.
    :param current_radius: The current distance of the rover from the GNSS point, gotten from current_info.
    Should be a float.
    :param desired_radius: The desired distance of the rover from the GNSS point, gotten from the current
    semicircle info. Should be a float.
    :return: Returns a boolean indicating if the rover is deflected or not.
    '''
    # We use a tolerance of +-0.3 meters. May modify later.
    if desired_radius - 0.3 > current_radius or desired_radius + 0.3 < current_radius:
        return True
    else:
        return False


def normalize_angle(angle):
    """Wrap angle to [-pi, pi]."""
    a = math.fmod(angle + math.pi, 2.0 * math.pi)
    if a < 0.0:
        a += 2.0 * math.pi
    return a - math.pi

def shortest_angular_error(theta_current, theta_goal):
    '''
    A way of knowing the shortest rotation path (positive or negative) for an object to be in a desired orientation.
    :param theta_current: current orientation of the rover in radians. Should be a float.
    :param theta_goal: desired orientation to rotate to in radians. Should be a float.
    :return: Returns an angle in radians from -pi to +pi. Float.
    '''
    theta_current = normalize_angle(theta_current)
    theta_goal = normalize_angle(theta_goal)
    diff = theta_goal - theta_current
    # This maps any angle to [-pi, pi]
    return math.atan2(math.sin(diff), math.cos(diff))


def go_to_tag(x_offset, distance, linear_velocity):
    '''
    This function moves the rover towards the tag, aligning the center of the screen with the center of the tag
    depending on the offset.
    :param x_offset: The distance in pixels from the center of the screen to the center of the tag. Float.
    :param distance: The real distance in meters from the camera to the center of the tag. Float.
    :param linear_velocity: The desired linear velocity which we want the rover to move forward in. Float.
    :return: returns the modified (or not) linear velocity and the angular velocity for correction of orientation.
    '''
    x_center = 620  # center of screen in pixels
    angular_vel = 0.0
    linear_vel = linear_velocity
    low_lin_vel = linear_vel / 2 #slower linear velocity for when we are close to the tag

    if 1.0 < distance < 2.5:
        linear_vel = low_lin_vel
    elif distance <= 1.0:
        linear_vel = 0.0

    # Check if we need orientation correction:
    if x_offset > x_center + 20:
        angular_vel = 1.0
        return linear_vel, angular_vel
    elif x_offset < x_center - 20:
        angular_vel = -1.0
        return linear_vel, angular_vel
    else:
        return linear_vel, angular_vel


def go_to_tag_undetected(time_since_undetection, current_time, last_off, last_dis):
    '''
    This function allows the rover to keep moving forward (when a tag has already been detected) even when the tag
    doesn't produce data for a few moments due to noise, for example.
    :param time_since_undetection: the exact time at which an already detected tag stopped producing data.
    :param current_time: the current time in miliseconds. (float)
    :param last_off:
    :param last_dis:
    :return:
    '''
    # This method is based upon pure estimation. No real data or facts are being used.
    time_elapsed = current_time - time_since_undetection
    linear_vel = 1.0
    angular_vel = 1.0

    r = linear_vel / angular_vel

    max_alignment_distance = (2 * math.pi * r) / 5    # focal width of camera is 150 degrees
                                                      # and rover will turn only in one direction
                                                      # so 75 deg, which is about 1/5 of circumference

    def estimation(last_offset, last_distance):
        last_offset = abs(last_offset)
        last_distance = abs(last_distance)

        if last_distance < 1.5:
            d = 0.0
        elif last_distance < 3:
            if 100 <last_offset < 300:
                d = max_alignment_distance * 0.4
            elif last_offset < 500:
                d = max_alignment_distance * 0.7
            elif last_offset >= 500:
                d = max_alignment_distance
            else:
                d = max_alignment_distance * 0.1
        elif last_distance < 5:
            if 200 < last_offset < 500:
                d = max_alignment_distance * 0.45
            elif last_offset >= 500:
                d = max_alignment_distance * 0.75
            else:
                d = max_alignment_distance * 0.1
        else:
            if last_offset > 300:
                d = max_alignment_distance * 0.6
            else:
                d = max_alignment_distance * 0.1

        t = d / linear_vel
        return t

    if time_elapsed < estimation(last_off, last_dis):
        if last_off < 0.0:
            angular_vel *= -1
            return linear_vel, angular_vel
        else:
            return linear_vel, angular_vel
    else:
        angular_vel = 0.0
        return linear_vel, angular_vel


class DriveLogic(Node):
    def __init__(self):
        super().__init__('drive_logic')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/swerve', 10)
        self.aruco_subscription = self.create_subscription(Float32MultiArray, 'aruco_results', self.aruco_callback, 10)
        self.localization_subscription = self.create_subscription(Float32MultiArray, 'localization_info', self.localization_callback, 10)
        self.imu_subscription = self.create_subscription(Float32MultiArray, "imu_info", self.imu_callback, 10)

        self.circle = "small"  #big
        self.sc_num = 1
        self.semicircle = circles[f"{self.circle}_circle"][f"semicircle_{self.sc_num}"]

        self.reference_point = []
        self.initial_re_point = []
        self.difference_x_pos = 0
        self.difference_y_pos = 0
        self.path_orientation = 0
        self.orientation_at_end_2nd_step = 0
        self.oriented_to_re_path = False
        self.got_to_re_point = False
        self.oriented_to_normal_path = False
        self.ended_reorientation = True

        self.full_search_done = False   # check if the full search has been done without finding a tag

        self.current_info = {}

        #self.current_time = 0.0
        self.start_time = self.get_clock().now()
        #self.timer = self.create_timer(0.01, self.update_time)  # 100 Hz

        self.ended_search_drive = False
        self.previous_offset = 0.0
        self.previous_dis = 0.0
        self.previous_time = 0.0

        self.aruco_msg = None
        self.localization_msg = None
        self.imu_msg = None

        #self.subscription  # prevent unused variable warning

    def aruco_callback(self, msg):
        self.aruco_msg = msg
        self.drive_callback()

    def localization_callback(self, msg):
        self.localization_msg = msg
        self.drive_callback()

    def imu_callback(self, msg):
        self.imu_msg = msg
        self.drive_callback()


    def drive_callback(self):
        # wait until we’ve received at least one message from each topic
        if (self.aruco_msg is None or
            self.localization_msg is None or
            self.imu_msg is None):
            return

        aruco = self.aruco_msg
        localization = self.localization_msg
        imu = self.imu_msg

        no_tag_found_yet = aruco.data[0] == -1.0 and aruco.data[1] == 0.0 and aruco.data[2] == 0.0 and self.ended_search_drive == False
        detecting_a_tag = aruco.data[0] != -1.0 or aruco.data[1] != 0.0 or aruco.data[2] != 0.0
        tag_found_but_not_being_detected = aruco.data[0] == -1.0 and aruco.data[1] == 0.0 and aruco.data[2] == 0.0 and self.ended_search_drive == True

        # BEFORE THIS CODE, TURN THE LOCALIZATION INFO (AND ORIENTATION FROM IMU) TO OUR LOCAL
        # COORDINATES (CENTER OF THE CIRCLE AS THE ORIGIN, AND INITIAL POSITION OF ROVER AS (radius, 0).
        actual_x_linear_vel, actual_y_linear_vel, actual_angular_vel, current_orientation = imu.data

        current_x_position, current_y_position = localization.data

        current_radius = math.sqrt((current_x_position ** 2) + (current_y_position ** 2))
        current_angle = math.atan2(current_y_position, current_x_position)

        if current_angle < 0:
            current_angle += 2 * math.pi

        self.current_info = {
            "x_position" : current_x_position,
            "y_position" : current_y_position,
            "angle" : current_angle,
            "orientation" : current_orientation,
            "radius" : current_radius,
            "linear_velocity" : actual_x_linear_vel
        }

        sw_msg = Float32MultiArray()

        # if-else block to decide between our 3 options: search drive, go to tag with tag detected,
        # or go to tag with tag undetected (when we loose the detection for a few seconds)
        # if no_tag_found_yet:
        #     self.circle, self.sc_num, self.full_search_done = get_current_semicircle(self.circle, self.semicircle, self.sc_num, self.current_info, self.full_search_done)
        #     self.semicircle = circles[f"{self.circle}_circle"][f"semicircle_{self.sc_num}"]

        #     points_list = semicircle_segmentation(self.semicircle, 1)

        #     lin_y = 0.0
        #     lin_x = 3.0
        #     ang_pos = 0.0 # self.current_info["linear_velocity"] / self.semicircle["radius"]  # publish the ang vel with actual lin vel
        #     ang_neg = 0.0

        #     deflected = deflection(self.current_info["radius"], self.semicircle["radius"])

        #     if deflected and self.ended_reorientation:  # This allows us to get the reference point just at the beginning of the reorientation
        #         self.ended_reorientation = False
        #         self.reference_point = get_reference_point(self.current_info, points_list, self.semicircle)
        #         print(self.reference_point)
        #         self.initial_re_point = self.current_info

        #         self.difference_x_pos = self.reference_point["x_position"] - self.current_info["x_position"]
        #         self.difference_y_pos = self.reference_point["y_position"] - self.current_info["y_position"]

        #         self.path_orientation = math.atan2(self.difference_y_pos, self.difference_x_pos)
        #         if self.path_orientation < 0:
        #             self.path_orientation += 2 * math.pi

        #     if not self.ended_reorientation:
        #         if not self.oriented_to_re_path:
        #             # These variables are for comparing if the expected initial rotation is the same as the current one.
        #             # If the current starts being on the opposite way, it's because it has already passed the desired angle,
        #             # so we should stop the rotation as soon as this happens
        #             initial_rotation_1 = shortest_angular_error(self.initial_re_point["orientation"],
        #                                                       self.path_orientation)
        #             current_rotation_1 = shortest_angular_error(self.current_info["orientation"],
        #                                                       self.path_orientation)

        #             if initial_rotation_1 < 0 and current_rotation_1 < 0:
        #                 ang_neg = 1.0
        #                 lin_x = 0.0
        #                 lin_y = 0.0
        #                 ang_pos = 0.0
        #             elif initial_rotation_1 > 0 and current_rotation_1 > 0:
        #                 ang_pos = 1.0
        #                 lin_x = 0.0
        #                 lin_y = 0.0
        #                 ang_neg = 0.0
        #             else:
        #                 self.oriented_to_re_path = True
        #                 print("\n\nORIENTED TO REORIENTATION PATH\n\n")

        #         elif self.oriented_to_re_path and not self.got_to_re_point:
        #             # Similar logic as previous block
        #             initial_rad_diff = self.initial_re_point["radius"] - self.semicircle["radius"]
        #             current_rad_diff = self.current_info["radius"] - self.semicircle["radius"]

        #             if (initial_rad_diff < 0 and current_rad_diff < 0) or (initial_rad_diff > 0 and current_rad_diff > 0):
        #                 lin_x = 1.0
        #                 lin_y = 0.0
        #                 ang_pos = 0.0
        #                 ang_neg = 0.0
        #             else:
        #                 print("\n\nGOT TO REORIENTATION POINT\n\n")
        #                 self.got_to_re_point = True
        #                 self.orientation_at_end_2nd_step = self.current_info["orientation"]

        #         elif self.oriented_to_re_path and self.got_to_re_point and not self.oriented_to_normal_path:
        #             #Similar logic as 2 previous blocks
        #             print("\n\nIN LAST REORIENTATION BLOCK\n\n")
        #             initial_rotation_2 = shortest_angular_error(self.orientation_at_end_2nd_step,
        #                                                       self.reference_point["orientation"])
        #             current_rotation_2 = shortest_angular_error(self.current_info["orientation"],
        #                                                       self.reference_point["orientation"])
        #             print(f"Initial diff = {initial_rotation_2}")
        #             print(f"current diff = {current_rotation_2}")

        #             if initial_rotation_2 < 0 and current_rotation_2 < 0:
        #                 print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")
        #                 ang_neg = 1.0
        #                 lin_x = 0.0
        #                 lin_y = 0.0
        #                 ang_pos = 0.0
        #             elif initial_rotation_2 > 0 and current_rotation_2 > 0:
        #                 ang_pos = 1.0
        #                 lin_x = 0.0
        #                 lin_y = 0.0
        #                 ang_neg = 0.0
        #             else:
        #                 print("\n\nORIENTED TO REFERENCE POINT\n\n")
        #                 self.oriented_to_normal_path = True

        #         else:
        #             # ended_reorientation true to not run the reorientation block next time.
        #             # Initialize the step to false for next possible reorientation.
        #             self.ended_reorientation = True
        #             self.oriented_to_re_path = False
        #             self.got_to_re_point = False
        #             self.oriented_to_normal_path = False
        #             print("ENDED REORIENTATION")

        #     else:
        #         if not self.full_search_done:
        #             lin_y = 0.0
        #             lin_x = 3.0
        #             ang_pos = self.current_info["linear_velocity"] / self.semicircle["radius"]  # publish the ang vel with actual lin vel
        #             ang_neg = 0.0
        #         else:
        #             lin_y = 0.0
        #             lin_x = 0.0
        #             ang_pos = 0.0
        #             ang_neg = 0.0

        #     sw_msg.data = [lin_y, lin_x, ang_pos, ang_neg]


        # elif detecting_a_tag:
        #     linear_velocity = 1.5
        #     linear_vel, angular_vel = go_to_tag(aruco.data[1], aruco.data[2], linear_velocity)

        #     self.ended_search_drive = True
        #     self.previous_offset = aruco.data[1]
        #     self.previous_dis = aruco.data[2]
        #     self.previous_time = self.get_clock().now()

        #     lin_y = linear_vel
        #     lin_x = 0.0
        #     ang_pos = angular_vel
        #     ang_neg = 0.0
        #     sw_msg.data = [lin_y, lin_x, ang_pos, ang_neg]

        # elif tag_found_but_not_being_detected:
        #     current_time = self.get_clock().now()

        #     linear_vel, angular_vel = go_to_tag_undetected(self.previous_time, current_time, self.previous_offset, self.previous_dis)

        #     lin_y = linear_vel
        #     lin_x = 0.0
        #     ang_pos = angular_vel
        #     ang_neg = 0.0
        #     sw_msg.data = [lin_y, lin_x, ang_pos, ang_neg]

        # self.publisher_.publish(sw_msg)
        # self.get_logger().info("Publishing swerve...")

        if no_tag_found_yet:

            lin_y = 0.0
            lin_x = 0.0
            ang_pos = 1.0  # self.current_info["linear_velocity"] / self.semicircle["radius"]  # publish the ang vel with actual lin vel
            ang_neg = 0.0

            sw_msg.data = [lin_y, lin_x, ang_pos, ang_neg]

        elif detecting_a_tag:
            linear_velocity = 1.5
            linear_vel, angular_vel = go_to_tag(aruco.data[1], aruco.data[2], linear_velocity)

            self.full_search_done = True
            self.ended_search_drive = True
            self.previous_offset = aruco.data[1]
            self.previous_dis = aruco.data[2]
            self.previous_time = self.get_clock().now()

            lin_y = linear_vel
            lin_x = 0.0
            ang_pos = angular_vel
            ang_neg = 0.0
            sw_msg.data = [lin_y, lin_x, ang_pos, ang_neg]

        elif tag_found_but_not_being_detected:
            current_time = self.get_clock().now()

            linear_vel, angular_vel = go_to_tag_undetected(self.previous_time, current_time, self.previous_offset,
                                                        self.previous_dis)

            lin_y = linear_vel
            lin_x = 0.0
            ang_pos = angular_vel
            ang_neg = 0.0
            sw_msg.data = [lin_y, lin_x, ang_pos, ang_neg]

        self.publisher_.publish(sw_msg)
        self.get_logger().info("Publishing swerve...")


def main(args=None):
    rclpy.init(args=args)
    drive_logic_publisher = DriveLogic()

    rclpy.spin(drive_logic_publisher)

    drive_logic_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()