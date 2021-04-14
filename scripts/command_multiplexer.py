#!/usr/bin/env python

import rospy
import sys

from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDrive

global current_command_topic
global offboard_command

car_name              = str(sys.argv[1])
listen_offboard       = str(sys.argv[2])
joy_angle_axis        = 2
joy_angle_scaler      = 1.0
joy_speed_axis        = 1
joy_speed_scaler      = 0.5
ctl_offboard_button   = 7
ctl_teleop_button     = 6
command_topic         = ['offboard/command',
                         'teleop/command']
log_message           = 'control priority assigned to - {}'
control_priority      = ['JTX2_OFFBOARD',
                         'REMOTE_LOGITECH']
current_command_topic = 'teleop/command'
message_display       = [False, False]
multiplexer_pub       = rospy.Publisher('/{}/multiplexer/command'.format(car_name), AckermannDrive, queue_size = 1)
offboard_command      = AckermannDrive()

def offboard_callback(data):
    global offboard_command
    offboard_command.steering_angle = -1.0 * data.steering_angle
    offboard_command.speed          = data.speed

def joy_command_callback(data):
    global current_command_topic
    global offboard_command
    passthrough_command = AckermannDrive()
    # listen to control transfer commands
    if data.buttons[ctl_offboard_button] and not data.buttons[ctl_teleop_button]:
        if not message_display[0]:
            rospy.loginfo(log_message.format(control_priority[0]))
            message_display[0] = True
            message_display[1] = False
        if current_command_topic != command_topic[0]:
            current_command_topic = command_topic[0]
    if not data.buttons[ctl_offboard_button] and data.buttons[ctl_teleop_button]:
        if not message_display[1]:
            rospy.loginfo(log_message.format(control_priority[1]))
            message_display[0] = False
            message_display[1] = True
        if current_command_topic != command_topic[1]:
            current_command_topic = command_topic[1]
    if current_command_topic == command_topic[1]:
        # identify and scale raw command data
        passthrough_command.steering_angle = data.axes[joy_angle_axis] * joy_angle_scaler
        passthrough_command.speed          = data.axes[joy_speed_axis] * joy_speed_scaler
    elif current_command_topic == command_topic[0] and listen_offboard == 'true':
        passthrough_command = offboard_command
    multiplexer_pub.publish(passthrough_command)

if __name__ == '__main__':
    try:
        rospy.init_node('command_multiplexer', anonymous = True)
        if listen_offboard == 'true':
            rospy.Subscriber('/{}/offboard/command'.format(car_name), AckermannDrive, offboard_callback)
        rospy.Subscriber('/{}/joy'.format(car_name), Joy, joy_command_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
