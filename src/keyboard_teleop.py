#!/usr/bin/env python

import rospy
import sys
import select
import termios

from ackermann_msgs.msg import AckermannDrive

car_name     = str(sys.argv[1])
speed        = 0.05
steer        = 0.25
key_bindings = {
    'w': (1, 0),
    'd': (1, -1),
    'a': (1, 1),
    's': (-1, 0),
}

def getKey():
   tty.setraw(sys.stdin.fileno())
   select.select([sys.stdin], [], [], 0)
   key = sys.stdin.read(1)
   termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
   return key

def vels(speed, turn):
  return 'currently:\tspeed %s\tturn %s ' % (speed,turn)

if __name__ == '__main__':

    rospy.init_node('keyboard_teleop', anonymous = True)

    settings = termios.tcgetattr(sys.stdin)
    command_pub = rospy.Publisher('/{}/command'.format(car_name), AckermannDrive, queue_size = 1)
    speed_now = 0
    steer_now = 0
    status    = 0

    try:
        while True:
           key = getKey()
           if key in key_bindings.keys():
              steer_now = key_bindings[key][0]
              speed_now = key_bindings[key][1]
           else:
              steer_now = 0.0
              speed_now = 0.0
              if (key == '\x03'):
                 break

           command = AckermannDrive();
           command.steering_angle = steer_now * steer
           command.speed = speed_now * speed
           command_pub.publish(command)

    except:
        print 'error: key bind'

    finally:
        command = AckermannDrive();
        command.steering_angle = steer_now * steer
        command.speed = speed_now * speed
        command_pub.publish(command)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
