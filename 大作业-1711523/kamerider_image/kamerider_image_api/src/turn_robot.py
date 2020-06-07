#!/usr/bin/env python
# -*- coding: utf-8 -*
import rospy
import roslib
from sensor_msgs.msg import RegionOfInterest
from geometry_msgs.msg import Twist

from std_msgs.msg import String
class turn_robot():
    def __init__(self):
        self.face_sub = rospy.Subscriber("/start_recognize_faces", String, self.imgCallback,queue_size=1)

        self.roi_sub = rospy.Subscriber('roi',RegionOfInterest, self.roiCallback,queue_size=1)
        self.cmd_pub = rospy.Publisher('cmd_vel_mux/input/navi',Twist,queue_size=1)
        self.time = 0
        self.time_turn = 2
    def roiCallback(self, msg):
	if flag==0:
            msg = RegionOfInterest()
            if msg.width == 0 and msg.height == 0:
                self.time +=1
            if self.time == self.time_turn:
                self.time = 0
                vel = Twist()
                vel.angular.z = 0.6
                self.cmd_pub.publish(vel)
    def imgCallback(self, msg):
	if msg.data=="ok":
	    flag=1
if __name__ == '__main__':
    rospy.init_node('turn_and_check', anonymous=True)
    turn = turn_robot()
    flag=0
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("CLOSE Turn")

