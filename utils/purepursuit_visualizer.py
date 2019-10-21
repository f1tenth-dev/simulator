#!/usr/bin/env python

import rospy
import sys

from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point32
from nav_msgs.msg import Odometry

car_name = str(sys.argv[1])

global wp_seq
global curr_polygon

wp_seq       = 0
curr_polygon = PolygonStamped()
frame_id     = 'map'
polygon_pub  = rospy.Publisher('/{}/purepursuit_control/visualize'.format(car_name), PolygonStamped, queue_size = 1)

global base_link
global nearest_pose
global nearest_goal

base_link    = Point32()
nearest_pose = Point32()
nearest_goal = Point32()

def pose_callback(data):
    global nearest_pose
    nearest_pose.x = data.pose.position.x
    nearest_pose.y = data.pose.position.y

def goal_callback(data):
    global nearest_goal
    nearest_goal.x = data.pose.position.x
    nearest_goal.y = data.pose.position.y

def odom_callback(data):
    global base_link
    base_link.x = data.pose.pose.position.x
    base_link.y = data.pose.pose.position.y
    publish_current_polygon()

def publish_current_polygon():
    global curr_polygon
    global wp_seq
    global base_link
    global nearest_pose
    global nearest_goal
    curr_polygon.header.frame_id = frame_id
    curr_polygon.polygon.points = [nearest_pose, base_link, nearest_goal]
    polygon_pub.publish(curr_polygon)
    curr_polygon.header.seq = wp_seq
    curr_polygon.header.stamp = rospy.Time.now()
    wp_seq = wp_seq + 1
    polygon_pub.publish(curr_polygon)

if __name__ == '__main__':
    try:
        rospy.init_node('purepursuit_visualizer', anonymous = True)
        rospy.Subscriber('/{}/purepursuit_control/ang_goal'.format(car_name), PoseStamped, goal_callback)
        rospy.Subscriber('/{}/purepursuit_control/visualize_nearest_point'.format(car_name), PoseStamped, pose_callback)
        rospy.Subscriber('/{}/base/odom'.format(car_name), Odometry, odom_callback)
        rospy.spin()
    except rospy.ROSException:
        pass
