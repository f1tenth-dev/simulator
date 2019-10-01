#!/usr/bin/env python

import rospy
import sys
import select
import termios
import tty

from ackermann_msgs.msg import AckermannDrive

keyBindings = {
  'w':(1,0),
  'd':(1,-1),
  'a':(1,1),
  's':(-1,0),
}

speed_limit = 0.50
angle_limit = 0.25

def getKey():
   tty.setraw(sys.stdin.fileno())
   select.select([sys.stdin], [], [], 0)
   key = sys.stdin.read(1)
   termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
   return key

def vels(speed, turn):
  return 'currently:\tspeed {}\tturn {}'.format(speed, turn)

if __name__=="__main__":
  settings = termios.tcgetattr(sys.stdin)
  command_pub = rospy.Publisher('/{}/command'.format(str(sys.argv[1])), AckermannDrive, queue_size = 1)
  rospy.init_node('keyboard_teleop', anonymous = True)

  speed  = 0
  angle  = 0
  status = 0

  try:
    while True:
       key = getKey()
       if key in keyBindings.keys():
          speed = keyBindings[key][0]
          angle = keyBindings[key][1]
       else:
          speed = 0
          angle = 0
          if (key == '\x03'):
             break
       command = AckermannDrive();
       command.speed = speed * speed_limit
       command.steering_angle = angle * angle_limit
       command_pub.publish(command)

  except:
    print 'error'

  finally:
    command = AckermannDrive();
    command.speed = speed * speed_limit
    command.steering_angle = angle * angle_limit
    command_pub.publish(command)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
