#!/usr/bin/env python


"""
    partybot.py - Version 0.2 2019-03-30
    
    A party robot to serve guests and entertainment.
    
"""

import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from sound_play.libsoundplay import SoundClient
import sys
from subprocess import call
from geometry_msgs.msg import Twist
from math import radians
import os
from turtlebot_msgs.srv import SetFollowState
from tf.transformations import quaternion_from_euler
# import loca
# from navigation import *
from ch3_pkg.msg import Num
# from navigation.msg import Loca

class PartyBot:
    def __init__(self, script_path):
        rospy.init_node('partybot')

        rospy.on_shutdown(self.cleanup)

        self.rate = rospy.Rate(1)

        self.soundhandle = SoundClient()

        rospy.sleep(1)

        # Make sure any lingering sound_play processes are stopped.
        self.soundhandle.stopAll()
        # self.soundhandle.say('Hello, I am PartyBot. Where need I go?')
        rospy.loginfo("Ready, waiting for commands...")

        # Subscribe to the recognizer output and set the callback function
        rospy.Subscriber('voiceWords', String, self.talkback)
        self.xf = rospy.Publisher('voiceWakeup',Bool,queue_size=1)
        # rospy.spin()

        self.workout = False
        self.outcome = rospy.Subscriber("finish",Bool,self.finish)
        self.passtag = 1
        self.location = rospy.Publisher("location", Num, queue_size=1)
        

        self.locations = dict()

        # Location A
        self.loca = Num()
        self.A_x = 0.0
        self.A_y = 0.0
        # A_theta = 0.0

        # self.quaternion = quaternion_from_euler(0.0, 0.0, A_theta)
        # rospy.loginfo("quaternion::::okookokokokkokokokokookkkkkkkkkkkkkkkkkkoookokokokookokkokooo----------")

        # self.locations['A'] = Pose(Point(self.A_x, self.A_y, 0.000), Quaternion(self.quaternion[0], self.quaternion[1], self.quaternion[2], self.quaternion[3]))
        # rospy.loginfo("okookokokokkokokokokookkkkkkkkkkkkkkkkkkoookokokokookokkokooo----------")
        rospy.sleep(5)
        self.soundhandle.say('Hello, I am PartyBot. Where need I go?')
        
        rospy.sleep(3)
        self.xf.publish(True)


    def finish(self,msg):
        self.workout = msg.data

    def talkback(self, msg):
        self.workout = False
        rospy.loginfo(msg.data)
        # self.xf.publish(False)
        self.passtag = 1

        if msg.data.find('kitchen') > -1:
            self.soundhandle.say("I heard you want me to go to the kitchen. Ok ,I will go there at once.")
            self.A_x = 3.5
            self.A_y = 0.0
            # rospy.sleep(10)
        elif msg.data.find('bedroom') > -1:
            self.soundhandle.say("I heard you want me to go to the bedroom. Ok ,I will go there at once.")
            self.A_x = -0.15
            self.A_y = 1.76
            # rospy.sleep(5)

        elif msg.data.find('living room') > -1:
            self.soundhandle.say("I heard you want me to go to the living room. Ok ,I will go there at once.")
            self.A_x = 0.62
            self.A_y = 0.0
            # rospy.sleep(5)
            
        elif msg.data.find('study room') > -1:
            self.soundhandle.say("I heard you want me to go to the study room. Ok ,I will go there at once.")
            self.A_x = 3.2
            self.A_y = 2.1

        else:
            self.soundhandle.say(
                "Well,I can go to kitchen, bedroom, living room, study room.")
            self.passtag = 0
        # else: self.soundhandle.say("Sorry, I cannot hear you clearly. Please say again.")
        rospy.sleep(5)
        if self.passtag:
            self.loca.A_x = self.A_x
            self.loca.A_y = self.A_y
            self.location.publish(self.loca)
            # self.locations['A'] = Pose(Point(self.A_x, self.A_y, 0.000), Quaternion(self.quaternion[0], self.quaternion[1], self.quaternion[2], self.quaternion[3]))
            # self.location.publish(self.locations['A'])

        while not self.workout and self.passtag:
            rospy.sleep(1)
        if self.passtag:
            self.soundhandle.say("I arrive at destination. Where need I go?")
            rospy.sleep(3)
        # rospy.sleep(1)
        self.xf.publish(True)
        

    def cleanup(self):
        self.soundhandle.stopAll()
        rospy.loginfo("Shutting down partybot node...")


if __name__ == "__main__":
    try:
        PartyBot(sys.path[0])
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Partybot node terminated.")
