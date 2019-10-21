#!/usr/bin/env python

import rospy
import sys
import os
import math
import csv

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int64
from std_msgs.msg import String

car_name            = str(sys.argv[1])
trajectory_name     = str(sys.argv[2])
adaptive_lookahead  = str(sys.argv[3])
ang_lookahead_dist  = float(sys.argv[4])

plan = []

# if ang_lookahead_dist > 5.0:
#     ang_lookahead_dist = 500
#     vel_lookahead_dist = 1000
# else:
#     ang_lookahead_dist = int(ang_lookahead_dist * 100)
#     vel_lookahead_dist = ang_lookahead_dist * 2

global plan_size
global seq

plan         = []
frame_id     = 'map'
seq          = 0
ang_goal_pub = rospy.Publisher('/{}/purepursuit_control/ang_goal'.format(car_name), PoseStamped, queue_size = 1)
vel_goal_pub = rospy.Publisher('/{}/purepursuit_control/vel_goal'.format(car_name), PoseStamped, queue_size = 1)
plan_size    = 0

def construct_path():
    global plan_size
    file_path = os.path.expanduser('~/catkin_ws/src/f1tenth_purepursuit/path/{}.csv'.format(trajectory_name))

    with open(file_path) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter = ',')
        for waypoint in csv_reader:
            plan.append(waypoint)

    for index in range(0, len(plan)):
        for point in range(0, len(plan[index])):
            plan[index][point] = float(plan[index][point])

    plan_size = len(plan)

# use adaptive lookahead as reported

brake_lookahead        = 2.00
caution_lookahead      = 2.50
unrestricted_lookahead = 3.00

global ang_lookahead_dist
global vel_lookahead_dist

if adaptive_lookahead != 'true':
    ang_lookahead_dist = int(float(sys.argv[4]) * 100)
    vel_lookahead_dist = ang_lookahead_dist * 2
else:
    ang_lookahead_dist = 100
    vel_lookahead_dist = 200

def dist_callback(data):
    global ang_lookahead_dist
    global vel_lookahead_dist

    if data.data == 'brake':
        ang_lookahead_dist = int(brake_lookahead * 100)
    elif data.data == 'caution':
        ang_lookahead_dist = int(caution_lookahead * 100)
    else:
        ang_lookahead_dist = int(unrestricted_lookahead * 100)

    vel_lookahead_dist = ang_lookahead_dist * 2

def pose_callback(data):
    global seq
    global plan_size
    global ang_lookahead_dist
    global vel_lookahead_dist

    # ang_lookahead_dist = int(rospy.get_param('lookahead_dist'))

    pose_index = data.data + ang_lookahead_dist

    if pose_index >= plan_size - 1:
        pose_index = pose_index - plan_size - 2

    goal                    = PoseStamped()
    goal.header.seq         = seq
    goal.header.stamp       = rospy.Time.now()
    goal.header.frame_id    = frame_id
    goal.pose.position.x    = plan[pose_index][0]
    goal.pose.position.y    = plan[pose_index][1]
    goal.pose.orientation.z = plan[pose_index][2]
    goal.pose.orientation.w = plan[pose_index][3]

    ang_goal_pub.publish(goal)

    pose_index = data.data + vel_lookahead_dist

    if pose_index > plan_size:
        pose_index = pose_index - plan_size

    # rospy.loginfo('current index: {}/{}'.format(pose_index, plan_size))

    goal                    = PoseStamped()
    goal.header.seq         = seq
    goal.header.stamp       = rospy.Time.now()
    goal.header.frame_id    = frame_id
    goal.pose.position.x    = plan[pose_index][0]
    goal.pose.position.y    = plan[pose_index][1]
    goal.pose.orientation.z = plan[pose_index][2]
    goal.pose.orientation.w = plan[pose_index][3]

    seq = seq + 1

    vel_goal_pub.publish(goal)

if __name__ == '__main__':
    try:
        rospy.init_node('nearest_goal_isolator', anonymous = True)
        # rospy.set_param('lookahead_dist', ang_lookahead_dist)
        if not plan:
            rospy.loginfo('obtaining trajectory')
            construct_path()
        if adaptive_lookahead == 'true':
            rospy.Subscriber('/{}/purepursuit_control/adaptive_lookahead'.format(car_name), String, dist_callback)
        rospy.Subscriber('/{}/purepursuit_control/index_nearest_point'.format(car_name), Int64, pose_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
