#!/usr/bin/env python

import rospy
import sys
import csv
import os

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry

trajectory_name = str(sys.argv[1])
sector_list     = str(sys.argv[2])

plan     = []
sectors  = []
frame_id = 'map'

def get_plan():
    file_path = os.path.expanduser('~/catkin_ws/src/f1tenth_purepursuit/path/{}.csv'.format(trajectory_name))
    with open(file_path) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter = ',')
        for waypoint in csv_reader:
            plan.append(waypoint)
    for index in range(0, len(plan)):
        for point in range(0, len(plan[index])):
            plan[index][point] = float(plan[index][point])

def get_sectors():
    file_path = os.path.expanduser('~/catkin_ws/src/f1tenth_purepursuit/sectors/{}.csv'.format(sector_list))
    with open(file_path) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter = ',')
        for sector in csv_reader:
            sectors.append(sector)
    for index in range(0, len(sectors)):
        sectors[index][0] = int(sectors[index][0])
        sectors[index][1] = int(sectors[index][1])
        sectors[index][2] = str(sectors[index][2])

class visualize_sector(object):

    def __init__(self, begin_index, end_index, sector_name, sector_type):
        self.begin_index = begin_index
        self.end_index   = end_index
        self.sector_type = sector_type
        self.sector_pub  = rospy.Publisher('/trajectory/{}'.format(sector_name), Path, queue_size = 1)
        self.path        = Path()
        self.seq         = 0

        self.construct()

    def construct(self):
        if not self.end_index < self.begin_index:
            for index in range(self.begin_index, self.end_index):
                waypoint = PoseStamped()
                waypoint.header.frame_id = frame_id
                waypoint.pose.position.x = plan[index][0]
                waypoint.pose.position.y = plan[index][1]
                waypoint.pose.orientation.z = plan[index][2]
                waypoint.pose.orientation.w = plan[index][3]
                self.path.poses.append(waypoint)
        else:
            for index in range(self.begin_index, len(plan)):
                waypoint = PoseStamped()
                waypoint.header.frame_id = frame_id
                waypoint.pose.position.x = plan[index][0]
                waypoint.pose.position.y = plan[index][1]
                waypoint.pose.orientation.z = plan[index][2]
                waypoint.pose.orientation.w = plan[index][3]
                self.path.poses.append(waypoint)
            for index in range(0, self.end_index):
                waypoint = PoseStamped()
                waypoint.header.frame_id = frame_id
                waypoint.pose.position.x = plan[index][0]
                waypoint.pose.position.y = plan[index][1]
                waypoint.pose.orientation.z = plan[index][2]
                waypoint.pose.orientation.w = plan[index][3]
                self.path.poses.append(waypoint)

    def visualize(self):
        self.path.header.seq      = self.seq
        self.path.header.stamp    = rospy.Time.now()
        self.path.header.frame_id = frame_id
        self.seq = self.seq + 1
        self.sector_pub.publish(self.path)

if __name__ == '__main__':
    try:
        rospy.init_node('trajectory_server', anonymous = True)
        get_plan()
        get_sectors()

        for index in range(0, len(sectors)):
            begin_index = sectors[index][0]
            end_index   = sectors[index][1]
            sector_name = 'sector_{}'.format(index + 1)
            exec('{} = visualize_sector(begin_index = {}, end_index = {}, sector_name = "{}", sector_type = "not_implemented")'.format(sector_name,
                                                                                                                                          begin_index,
                                                                                                                                          end_index,
                                                                                                                                          sector_name))

        while not rospy.is_shutdown():
            for index in range(0, len(sectors)):
                exec('sector_{}.visualize()'.format(index + 1))

            rospy.Rate(2).sleep()

    except rospy.ROSInterruptException:
        pass
