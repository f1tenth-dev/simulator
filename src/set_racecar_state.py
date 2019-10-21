#!/usr/bin/env python

import rospy
import sys

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState

if __name__ == '__main__':
    try:
        car_name  = str(sys.argv[1])
        pos_tuple = str(sys.argv[2])[1:-1].split(',')
        vel_yuple = str(sys.argv[3])[1:-1].split(',')
        frame_id  = 'map'
        rospy.init_node('teleport_node', anonymous = True)

        teleport_pub  = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size = 1)
        teleport_info = ModelState()

        teleport_info.pose.position.x = float(pos_tuple[0]) + 2.5
        teleport_info.pose.position.y = float(pos_tuple[1]) - 7.5
        teleport_info.pose.position.z = float(pos_tuple[2])

        teleport_info.twist.linear.x = float(vel_tuple[0])
        teleport_info.twist.linear.y = float(vel_tuple[1])
        teleport_info.twist.linear.z = float(vel_tuple[2])

        teleport_info.model_name = car_name
        teleport_info.reference_frame = frame_id

        teleport_pub.publish(teleport_info)
        rospy.Rate(1).sleep()

    except rospy.ROSInterruptException:
        pass
