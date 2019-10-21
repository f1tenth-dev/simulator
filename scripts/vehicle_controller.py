#!/usr/bin/env python

import rospy
import math
import tf
import sys
import tf2_ros
import tf2_geometry_msgs

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from ackermann_msgs.msg import AckermannDrive
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_msgs.msg import Float64

# car name

car_name            = str(sys.argv[1])
use_ackermann_model = str(sys.argv[2])
adaptive_lookahead  = str(sys.argv[3])

# global goal recall variables

global ang_goal_x
global ang_goal_y

global vel_goal_x
global vel_goal_y

ang_goal_x = 0.0
ang_goal_y = 0.0

vel_goal_x = 0.0
vel_goal_y = 0.0

# steering map constants

ANGLE_TURN_MAX      = 1.0
ANGLE_TURN_SWITCH_A = 0.50
ANGLE_TURN_SWITCH_B = 0.25
ANGLE_TURN_SWITCH_C = 0.12
ANGLE_TURN_MIN      = 0.0

# throttle map constants

SPEED_TURN_MAX      = 0.50
SPEED_TURN_SWITCH_A = 0.65
SPEED_TURN_SWITCH_B = 0.75
SPEED_TURN_SWITCH_C = 0.85
SPEED_TURN_MIN      = 1.0

# command to steering map constants

ANGLE_RANGE_A       = 45.0 # 60.0
ANGLE_RANGE_B       = 30.0
ANGLE_RANGE_C       = 15.0
ANGLE_RANGE_D       = 5.0

# velocity control

MAX_VEL_GOAL_DIST   = 6.0

# vehicle physical parameters

WHEELBASE_LEN       = 0.325

# adaptive speed control based on lookahead distance

SCALE_VEL_BRAKE        = 0.65
SCALE_VEL_CAUTION      = 0.80
SCALE_VEL_UNRESTRICTED = 1.00
SCALE_VEL_NO_ADAPTIVE_LOOKAHEAD = float(sys.argv[4])

# interprocess control string

GOAL_RIGHT          = "goal_to_right"
GOAL_LEFT           = "goal_to_left"
GOAL_ON_LINE        = "goal_on_line"

# loginfo msg types

MSG_A               = "goal at {}m"
MSG_B               = "goal at {}m bearing {} {}"
MSG_GOAL            = "recieved new goal: ({}, {})"

# command publisher

command_pub = rospy.Publisher('/{}/command'.format(car_name), AckermannDrive, queue_size = 1)

# deviation publisher

# deviation_pub = rospy.Publisher('/{}/deviation'.format(car_name), Float64, queue_size = 1)

# get front axle coordinates w.r.t map

front_axle               = Pose()
front_axle.position.x    = 0.325
front_axle.orientation.w = 1.0

# pure pursuit node

def vehicle_control_node(data):

    command = AckermannDrive()

#     log_dev = Float64()

    global ang_goal_x
    global ang_goal_y

    global vel_goal_x
    global vel_goal_y

    global lookahead_state

    curr_x = data.pose.pose.position.x
    curr_y = data.pose.pose.position.y

    heading = tf.transformations.euler_from_quaternion((data.pose.pose.orientation.x,
                                                        data.pose.pose.orientation.y,
                                                        data.pose.pose.orientation.z,
                                                        data.pose.pose.orientation.w))[2]

    # begin test (include wheel base)

    # begin logging range

#     log_range = math.sqrt(math.pow(ang_goal_x - curr_x, 2) + math.pow(ang_goal_y - curr_y, 2))

    # end logging range

    if use_ackermann_model == 'true':

        front_axle_x = (WHEELBASE_LEN * math.cos(heading)) + curr_x
        front_axle_y = (WHEELBASE_LEN * math.sin(heading)) + curr_y

        # rospy.loginfo('axle shift: {}'.format(math.sqrt(math.pow(curr_x - front_axle_x, 2) + math.pow(curr_y - front_axle_y, 2))))
        # rospy.loginfo('front axle: {}, {}'.format(round(front_axle_x, 2), round(front_axle_y, 2)))

        curr_x = front_axle_x
        curr_y = front_axle_y

    # end test

    eucl_d = math.sqrt(math.pow(ang_goal_x - curr_x, 2) + math.pow(ang_goal_y - curr_y, 2))

    # begin curvature test

    curvature = math.degrees(2.0*(abs(ang_goal_x) - abs(curr_x))/(math.pow(eucl_d, 2)))

    steering_angle = math.atan(curvature * WHEELBASE_LEN)

    # rospy.loginfo('steering from curvature: {}'.format(math.degrees(steering_angle)))

    # end curvature test

    theta  = math.atan2(ang_goal_y - curr_y, ang_goal_x - curr_x)

    proj_x = eucl_d * math.cos(heading) + curr_x
    proj_y = eucl_d * math.sin(heading) + curr_y

    proj_eucl_shift = math.sqrt(math.pow(proj_x - ang_goal_x, 2) + math.pow(proj_y - ang_goal_y, 2))

    angle_error = math.acos(1 - (math.pow(proj_eucl_shift, 2)/(2 * math.pow(eucl_d, 2))))
    angle_error = math.degrees(angle_error)

    goal_sector = (ang_goal_x - curr_x)*(proj_y - curr_y) - (ang_goal_y - curr_y)*(proj_x - curr_x)

    if goal_sector > 0:
        goal_sector = GOAL_RIGHT
    elif goal_sector < 0:
        goal_sector = GOAL_LEFT
    else:
        goal_sector = GOAL_ON_LINE

    # begin logging deviation

#     log_deviation = log_range * math.cos(math.radians(angle_error))

#     if goal_sector < 0:
#         log_deviation = -1.0 * log_deviation

#     log_dev.data = log_deviation

#     deviation_pub.publish(log_dev)

    # end logging deviation

    if goal_sector == GOAL_ON_LINE:
        msg = MSG_A.format(round(eucl_d, 2))
    else:
        msg = MSG_B.format(round(eucl_d, 2), round(angle_error, 2), goal_sector)
    # rospy.loginfo(msg)

    '''
    if angle_error > ANGLE_RANGE_A:
        command.steering_angle = ANGLE_TURN_MAX
        command.speed          = SPEED_TURN_MAX
    elif angle_error <= ANGLE_RANGE_A and angle_error > ANGLE_RANGE_B:
        command.steering_angle = ANGLE_TURN_SWITCH_A
        command.speed          = SPEED_TURN_SWITCH_A
    elif angle_error <= ANGLE_RANGE_B and angle_error > ANGLE_RANGE_C:
        command.steering_angle = ANGLE_TURN_SWITCH_B
        command.speed          = SPEED_TURN_SWITCH_B
    elif angle_error <= ANGLE_RANGE_C and angle_error > ANGLE_RANGE_D:
        command.steering_angle = ANGLE_TURN_SWITCH_C
        command.speed          = SPEED_TURN_SWITCH_C
    else:
        command.steering_angle = ANGLE_TURN_MIN
        command.speed          = SPEED_TURN_MIN
    '''

    # full P-control for angle and speed

    if angle_error > ANGLE_RANGE_A:
        angle_error = ANGLE_RANGE_A

    command.steering_angle = angle_error/ANGLE_RANGE_A
    # command.speed          = 1.0 - (angle_error/ANGLE_RANGE_A) # * (SPEED_TURN_MIN - SPEED_TURN_MAX)
    # command.speed          = command.speed + SPEED_TURN_MAX

    if goal_sector == GOAL_RIGHT:
        command.steering_angle = -1.0 * command.steering_angle

    # velocity control node

    vel_eucl_d = math.sqrt(math.pow(vel_goal_x - curr_x, 2) + math.pow(vel_goal_y - curr_y, 2))

    if use_ackermann_model == 'true':
        command.speed = (vel_eucl_d/(MAX_VEL_GOAL_DIST - WHEELBASE_LEN)) * SPEED_TURN_MIN
    else:
        command.speed = (vel_eucl_d/MAX_VEL_GOAL_DIST) * SPEED_TURN_MIN

    if adaptive_lookahead == 'true':
        if lookahead_state == 'brake':
            command.speed = SCALE_VEL_BRAKE * command.speed
        elif lookahead_state == 'caution':
            command.speed = SCALE_VEL_CAUTION * command.speed
        else:
            command.speed = SCALE_VEL_UNRESTRICTED * command.speed
    else:
        command.speed = SCALE_VEL_NO_ADAPTIVE_LOOKAHEAD * command.speed

    if command.speed < SPEED_TURN_MAX:
        command.speed = SPEED_TURN_MAX
    if command.speed > SPEED_TURN_MIN:
        command.speed = SPEED_TURN_MIN

    command_pub.publish(command)

# relative pose callback

def ang_pose_callback(data):

    global ang_goal_x
    global ang_goal_y

    ang_goal_x = data.pose.position.x
    ang_goal_y = data.pose.position.y

def vel_pose_callback(data):

    global vel_goal_x
    global vel_goal_y

    vel_goal_x = data.pose.position.x
    vel_goal_y = data.pose.position.y

# lookahead speed scaler

global lookahead_state

lookahead_state = 'caution'

def dist_callback(data):

    global lookahead_state

    lookahead_state = data.data

if __name__ == '__main__':
    try:
        rospy.init_node('vehicle_control_node', anonymous = True)

        rospy.Subscriber('/{}/base/odom'.format(car_name), Odometry, vehicle_control_node)

        if adaptive_lookahead == 'true':
            rospy.Subscriber('/{}/purepursuit_control/adaptive_lookahead'.format(car_name), String, dist_callback)

        rospy.Subscriber('/{}/purepursuit_control/ang_goal'.format(car_name), PoseStamped, ang_pose_callback)
        rospy.Subscriber('/{}/purepursuit_control/vel_goal'.format(car_name), PoseStamped, vel_pose_callback)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
