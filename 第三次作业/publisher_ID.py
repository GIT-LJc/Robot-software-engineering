#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def talker():
	pub = rospy.Publisher('my_id',String,queue_size=1)
	rospy.init_node('id_talker',anonymous=True)
	rate=rospy.Rate(1)
	number = 0
	
	while not rospy.is_shutdown():
		id_number="1711523 %s" % number
		rospy.loginfo(id_number)
		pub.publish(id_number)
		rate.sleep()
		number+=1

if __name__=='__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
