#!/usr/bin/env python

import rospy

from geometry_msgs.msg import PoseStamped

car_1_goal_pub = rospy.Publisher('/car_1/move_base_simple/goal', PoseStamped, queue_size = 1)
car_2_goal_pub = rospy.Publisher('/car_2/move_base_simple/goal', PoseStamped, queue_size = 1)
car_3_goal_pub = rospy.Publisher('/car_3/move_base_simple/goal', PoseStamped, queue_size = 1)

def goal_callback(data):
    car_1_goal_pub.publish(data)
    car_2_goal_pub.publish(data)
    car_3_goal_pub.publish(data)

if __name__ == '__main__':
    try:
        rospy.init_node('share_set_goal', anonymous = True)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, goal_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
