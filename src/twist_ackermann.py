#!/usr/bin/env python

import rospy
import sys
import math

from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Twist

car_name  = str(sys.argv[1])
wheelbase = 0.315

command_pub = rospy.Publisher('/{}/command'.format(car_name), AckermannDrive, queue_size = 1)

def twist_ackermann_node(data):
    command                = AckermannDrive()
    if data.linear.x == 0.0 or data.angular.z == 0.0:
        command.steering_angle = 0.0
    else:
        radius = data.linear.x/data.angular.z
        command.steering_angle = math.atan(wheelbase/radius)
    command.speed          = data.linear.x
    command_pub.publish(command)

if __name__ == '__main__':
    try:
        rospy.init_node('twist_ackermann_node', anonymous = True)
        rospy.Subscriber('/{}/cmd_vel'.format(car_name), Twist, twist_ackermann_node)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
