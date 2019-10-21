#!/usr/bin/env python

import rospy
import sys
import os
import math
import csv

from std_msgs.msg import Int64
from std_msgs.msg import String

car_name         = str(sys.argv[1])
sector_list_name = str(sys.argv[2])

sectors = []

decision_pub  = rospy.Publisher('/{}/purepursuit_control/adaptive_lookahead'.format(car_name), String, queue_size = 1)

def construct_path():
    file_path = os.path.expanduser('~/catkin_ws/src/f1tenth_purepursuit/sectors/{}.csv'.format(sector_list_name))

    with open(file_path) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter = ',')
        for sector in csv_reader:
            sectors.append(sector)

    for sector in range(0, len(sectors)):
            sectors[sector][0] = int(sectors[sector][0])
            sectors[sector][1] = int(sectors[sector][1])

def pose_callback(data):

    curr_pose_index = data.data
    adaptive_state  = String()
    curr_pose_state = 'caution'

    for index in range(0, len(sectors)):
        if sectors[index][1] < sectors[index][0]:
            if curr_pose_index > sectors[index][0] or curr_pose_index < sectors[index][1]:
                curr_pose_state = str(sectors[index][2])
                # rospy.loginfo('current lookahead state: {}'.format(sectors[index][2]))
        else:
            if curr_pose_index > sectors[index][0] and curr_pose_index < sectors[index][1]:
                curr_pose_state = str(sectors[index][2])
                # rospy.loginfo('current lookahead state: {}'.format(sectors[index][2]))

    adaptive_state.data = curr_pose_state
    decision_pub.publish(adaptive_state)

if __name__ == '__main__':
    try:
        rospy.init_node('adaptive_lookahead_node', anonymous = True)
        construct_path()
        rospy.Subscriber('/{}/purepursuit_control/index_nearest_point'.format(car_name), Int64, pose_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
