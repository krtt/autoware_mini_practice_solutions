#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

rospy.init_node('publisher')

pub = rospy.Publisher('/message', String, queue_size=10)

message = rospy.get_param('~message', "Hello World - default")
rate = rospy.get_param('~rate', 2)

rate = rospy.Rate(rate)

while not rospy.is_shutdown():
    pub.publish(message)
    rate.sleep()
