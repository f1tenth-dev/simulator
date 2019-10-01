#!/usr/bin/env python

import rospy
import sys

from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDriveBoost

car_name    = str(sys.argv[1])

command_pub = rospy.Publisher('/{}/command'.format(car_name), AckermannDriveBoost, queue_size = 1)

joy_angle_axis = 2
joy_speed_axis = 1
boost_button   = 7

def command_callback(data):
    command = AckermannDriveBoost()
    command.steering_angle = data.axes[joy_angle_axis]
    command.speed          = data.axes[joy_speed_axis]
    command.boost          = data.buttons[boost_button]
    command_pub.publish(command)

if __name__ == '__main__':
    try:
        rospy.init_node('{}_remote_control'.format(car_name), anonymous = True)
        rospy.Subscriber('/joy', Joy, command_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
