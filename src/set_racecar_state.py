#!/usr/bin/env python3

import rospy
import rospkg
import sys
import ast

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Point
from std_srvs.srv import Empty

car_name         = str(sys.argv[1])
acceptable_reset = ['true', 'True', '1', '1.0']
loop_quit        = False

# Get car init pose as offset
init_pose = Point()
init_pose_param = ast.literal_eval(rospy.get_param('/{}/init_pose'.format(car_name)))
init_pose.x, init_pose.y, init_pose.z = init_pose_param

# x_pos          = float(sys.argv[2])
# y_pos          = float(sys.argv[3])
# z_pos          = float(sys.argv[4])
# x_vel          = float(sys.argv[5])
# y_vel          = float(sys.argv[6])
# z_vel          = float(sys.argv[7])
car_1_reset_pose = [-9.0, -5.0,  0.0]
car_2_reset_pose = [-7.0, -5.0,  0.0]
car_3_reset_pose = [-5.0, -5.0,  0.0]
car_4_reset_pose = [-3.0, -5.0,  0.0]
car_5_reset_pose = [-1.0, -5.0,  0.0]
car_6_reset_pose = [ 1.0, -5.0,  0.0]
car_7_reset_pose = [ 3.0, -5.0,  0.0]
car_8_reset_pose = [ 5.0, -5.0,  0.0]
frame_id         = 'odom'

rospy.set_param('/{}/reset_to_pit_stop'.format(car_name), 'False')

def racecar_reset_state(req):
    
    state_msg = ModelState()
    state_msg.model_name = car_name
    state_msg.pose.position = init_pose
    state_msg.pose.orientation.x = 0.0
    state_msg.pose.orientation.y = 0.0
    state_msg.pose.orientation.z = 0.0
    state_msg.pose.orientation.w = 1.0

    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state(state_msg)

    except rospy.ServiceException as error_msg:
        print("Service call failed: %s" % error_msg)

    return True

if __name__ == '__main__':
    try:
        rospy.init_node('racecar_reset_pit_stop', anonymous = True)
        reset_srv = rospy.Service('/{}/reset_to_pit_stop'.format(car_name), Empty, racecar_reset_state)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
