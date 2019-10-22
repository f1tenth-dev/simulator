#!/usr/bin/env python

import rospy
import sys

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState

if __name__ == '__main__':
    try:
        car_name  = str(sys.argv[1])
        x_pos     = float(sys.argv[2])
        y_pos     = float(sys.argv[3])
        z_pos     = float(sys.argv[4])
        x_vel     = float(sys.argv[5])
        y_vel     = float(sys.argv[6])
        z_vel     = float(sys.argv[7])
        frame_id  = 'map'

        rospy.init_node('teleport_node', anonymous = True)

        teleport_pub  = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size = 1)
        teleport_info = ModelState()
        pose          = Pose()
        twist         = Twist()

        pose.position.x = x_pos
        pose.position.y = y_pos
        pose.position.z = z_pos

        twist.linear.x  = x_vel
        twist.linear.y  = y_vel
        twist.linear.z  = z_vel

        teleport_info.model_name      = car_name
        teleport_info.pose            = pose
        teleport_info.twist           = twist
        teleport_info.reference_frame = frame_id

        print('setting model state now')
        print(teleport_pub)
        print(teleport_info)

        while not rospy.is_shutdown():
            teleport_pub.publish(teleport_info)
            rospy.Rate(0.2).sleep()

        print('process completed')

    except rospy.ROSInterruptException:
        pass
