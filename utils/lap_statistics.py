#!/usr/bin/env python

import rospy
import sys
import math
import os
import csv

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PolygonStamped

car_name    = str(sys.argv[1])
number_laps = int(sys.argv[2])

global lap_count

lap_count    = 0

lap_resoultion = 0.1

flag_box = [[0.00,  0.85, 0.00],
            [0.10,  0.85, 0.00],
            [0.10, -1.70, 0.00],
            [0.00, -1.70, 0.00]]

side_A = Point32()
side_B = Point32()
side_C = Point32()
side_D = Point32()

[side_A.x, side_A.y, side_A.z] = flag_box[0]
[side_B.x, side_B.y, side_B.z] = flag_box[1]
[side_C.x, side_C.y, side_C.z] = flag_box[2]
[side_D.x, side_D.y, side_D.z] = flag_box[3]

flag_area                 = PolygonStamped()
flag_area.header.frame_id = 'map'
flag_area.polygon.points  = [side_A, side_B, side_C, side_D]

global seq
global prev_lap_time
global in_flag_area

seq           = 0
prev_lap_time = 0.0
in_flag_area  = False
flag_viz_pub  = rospy.Publisher('/{}/lap_statics'.format(car_name), PolygonStamped, queue_size = 1)

global prev_x
global prev_y

prev_x = 0.0
prev_y = 0.0

global total_dist
global inst_speed
global max_inst_speed

total_dist     = 0.0
inst_speed     = 0.0
max_inst_speed = 0.0

global prev_time
global inst_speed_list

prev_time       = 0.0
inst_speed_list = []

# file_path  = os.path.expanduser('~/catkin_ws/src/f1tenth_logging/logs/statistics/lap_statistics.csv')
# log_file   = open(file_path, mode = 'w')
# csv_writer = csv.writer(log_file, delimiter = ',', quoting = csv.QUOTE_NONNUMERIC)

global lap_data

lap_data = []

def odom_callback(data):
        global seq
        global prev_lap_time
        global in_flag_area
        global prev_x
        global prev_y
        global total_dist
        global inst_speed
        global max_inst_speed
        global prev_time
        global inst_speed_list
        global lap_count
        global lap_data

        flag_area.header.seq = seq
        seq = seq + 1
        flag_area.header.stamp = rospy.Time.now()
        flag_viz_pub.publish(flag_area)

        curr_x = data.pose.pose.position.x
        curr_y = data.pose.pose.position.y

        curr_x_vel = data.twist.twist.linear.x
        curr_y_vel = data.twist.twist.linear.y

        if curr_x >= flag_box[0][0] and curr_x <= flag_box[1][0]:
            if curr_y <= flag_box[1][1] and curr_y >= flag_box[2][1]:
                if not in_flag_area:
                    in_flag_area = True
                    if prev_lap_time != 0.0:
                        curr_time = rospy.Time.now().to_sec()
                        print('current lap time: {}'.format(curr_time - prev_lap_time))
                        # avg_speed = (total_dist/(curr_time - prev_lap_time))
                        avg_speed = 0.0
                        for inst_speed in inst_speed_list:
                            avg_speed = avg_speed + inst_speed
                        avg_speed = avg_speed/len(inst_speed_list)
                        print('lap distance: {} m, lap avg_speed:{} m/s, lap max_speed: {} m/s'.format(round(total_dist, 2),
                                                                                                       round(avg_speed, 2),
                                                                                                       round(max_inst_speed, 2)))
                        lap_data.append([round(curr_time - prev_lap_time, 2),
                                         round(total_dist, 2),
                                         round(avg_speed, 2),
                                         round(max_inst_speed, 2)])
                        print('\n')
                        prev_lap_time = curr_time
                        total_dist = 0.0
                        max_inst_speed = 0.0
                        inst_speed_list = []
                        lap_count = lap_count + 1

                    else:
                        curr_time = rospy.Time.now().to_sec()
                        print('first lap!')
                        print('statistics unavailable')
                        lap_data.append(['lap time (s)',
                                         'lap distance (m)',
                                         'lap avg_speed (m/s)',
                                         'lap max_speed (m/s)'])
                        print('\n')
                        prev_lap_time = curr_time
                        total_dist = 0.0
                        max_inst_speed = 0.0
                        inst_speed_list = []
                        lap_count = lap_count + 1

                if lap_count > number_laps:
                    print('finished collecting data')
                    file_path  = os.path.expanduser('~/catkin_ws/src/f1tenth_logging/logs/statistics/lap_statistics.csv')
                    log_file   = open(file_path, mode = 'w')
                    csv_writer = csv.writer(log_file, delimiter = ',', quoting = csv.QUOTE_NONNUMERIC)
                    for data in lap_data:
                        csv_writer.writerow(data)
                    log_file.close()
                    rospy.signal_shutdown('stopping statistics node')
            else:
                if in_flag_area:
                    in_flag_area = False
        else:
            if in_flag_area:
                in_flag_area = False

        # curr_time = rospy.Time.now().to_sec()

        # if not (curr_time - prev_time == 0.0):
        #    inst_speed = eucl_d/(curr_time - prev_time)
        # total_dist = total_dist + eucl_d

        eucl_x = math.pow(curr_x - prev_x, 2)
        eucl_y = math.pow(curr_y - prev_y, 2)
        eucl_d = math.sqrt(eucl_x + eucl_y)

        if eucl_d >= lap_resoultion:
            vel_eucl_x = math.pow(curr_x_vel, 2)
            vel_eucl_y = math.pow(curr_y_vel, 2)
            vel_eucl_d = math.sqrt(vel_eucl_x + vel_eucl_y)

            total_dist = total_dist + eucl_d

            inst_speed_list.append(vel_eucl_d)

            if vel_eucl_d > max_inst_speed:
                max_inst_speed = vel_eucl_d

            prev_x    = curr_x
            prev_y    = curr_y
            # prev_time = curr_time

if __name__ == '__main__':
    try:
        rospy.init_node('lap_statics', anonymous = True)
        rospy.Subscriber('/{}/base/odom'.format(car_name), Odometry, odom_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
