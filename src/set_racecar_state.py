#!/usr/bin/env python

import rospy
import rospkg
import sys

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

car_name         = str(sys.argv[1])
acceptable_reset = ['true', 'True', '1', '1.0']
loop_quit        = False
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

def racecar_reset_state():
    rospy.init_node('racecar_reset_pit_stop', anonymous = True)

    state_msg = ModelState()
    state_msg.model_name = car_name
    exec('state_msg.pose.position.x = {}_reset_pose[0]'.format(car_name))
    exec('state_msg.pose.position.y = {}_reset_pose[1]'.format(car_name))
    exec('state_msg.pose.position.z = {}_reset_pose[2]'.format(car_name))
    state_msg.pose.orientation.x = 0.0
    state_msg.pose.orientation.y = 0.0
    state_msg.pose.orientation.z = 0.0
    state_msg.pose.orientation.w = 0.0

    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state(state_msg)

    except rospy.ServiceException, error_msg:
        print("Service call failed: %s" % error_msg)

if __name__ == '__main__':
    try:
        while not loop_quit:
            if str(rospy.get_param('/{}/reset_to_pit_stop'.format(car_name))) in acceptable_reset:
                racecar_reset_state()
                rospy.set_param('/{}/reset_to_pit_stop'.format(car_name), 'False')
    except rospy.ROSInterruptException:
        pass
