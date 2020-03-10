#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def callback(data):
	rospy.loginfo(rospy.get_caller_id()+"I heared %s", data.data)

def subscribe_ID():
	rospy.init_node('subscribe_ID',anonymous=True)
	rospy.Subscriber("my_id", String, callback)
	rospy.spin()

if __name__=='__main__':
	subscribe_ID()
