#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
    partybot.py - Version 0.2 2019-03-30
    
    A party robot to serve guests and entertainment.
    
"""

import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from sound_play.libsoundplay import SoundClient
import sys
from subprocess import call
from geometry_msgs.msg import Twist
from math import radians
import os
from turtlebot_msgs.srv import SetFollowState
from cv_bridge import CvBridge, CvBridgeError
from opencv_apps.msg import RectArrayStamped

class PartyBot:
    def __init__(self, script_path):
        rospy.init_node('partybot')

        rospy.on_shutdown(self.cleanup)

        self.rate = rospy.Rate(1)

        # Create the sound client object
        self.soundhandle = SoundClient()
        # self.soundhandle = SoundClient(blocking=True)

        # Wait a moment to let the client connect to the sound_play server
        rospy.sleep(1)

        # Make sure any lingering sound_play processes are stopped.
        self.soundhandle.stopAll()
        # self.soundhandle.say("Ready")

        rospy.loginfo("Ready, waiting for commands...")
        # self.soundhandle.say('Hello, I am PartyBot. What can I do for you?')
        rospy.sleep(5)

        # Subscribe to the recognizer output and set the callback function
        rospy.Subscriber('voiceWords', String, self.talkback)
        self.xf = rospy.Publisher('voiceWakeup',Bool,queue_size=10)
        self.take_photo = rospy.Publisher("take_photo", Bool, queue_size=10)   #加入指引照相功能，判断检测效果
        self.body_signal = rospy.Publisher("body_signal",Bool, queue_size=10)
        self.detection_signal = rospy.Publisher("detection_signal",Bool, queue_size=10)
        self.recognization_signal = rospy.Publisher("recognization_signal",Bool, queue_size=10)
        self.record_signal = rospy.Publisher("record_signal", Bool, queue_size=10)
        self.camshift = rospy.Publisher("camshift",Bool,queue_size=10)
        self.object = rospy.Publisher("object",Bool,queue_size=10)

        # subscriber
        self.workout = False
        self.pede = 1
        self.outcome = rospy.Subscriber("finish",Bool,self.finish)
        self.detection_result = rospy.Subscriber("detection_result", String )
        self.recognization_result = rospy.Subscriber("recognization_result", String )
        self.record_result = rospy.Subscriber("record_result", String, self.response)
        rospy.Subscriber("/people_detect/found", RectArrayStamped ,self.people_find)
        self.passtag = 1

        rospy.sleep(2)
        # self.xf.publish(True)
        self.body_signal.publish(True)
        rospy.spin()
        

    def people_find(self,msg):
        if self.pede == 1:
            if msg.rects:
                self.pede = 0
                self.xf.publish(False)
                self.body_signal.publish(False)
                self.soundhandle.say('Hello, I am PartyBot. What can I do for you?')
                rospy.sleep(5)
                
                self.xf.publish(True)
            
        # else:
        #     self.body_signal.publish(True)

    def finish(self,msg):
        self.workout = msg.data

    def response(self, msg):
        self.soundhandle.say(msg.data)
        rospy.sleep(5)

    def talkback(self, msg):
        rospy.loginfo(msg.data)
        # self.xf.publish(False)
        self.workout = False

        if msg.data.find('introduce yourself') > -1:
            self.soundhandle.say(
                "I heard you want me to introduce myself. I am PartyBot. I am a party robot to serve you and have fun.")

        elif msg.data.find('take a photo') > -1:
            self.soundhandle.say("You want to take a photo? Maybe you need several seconds to prepare.")
            rospy.sleep(7)
            self.take_photo.publish(True)
            self.detection_signal.publish(True)

        elif msg.data.find('record') > -1:
            self.soundhandle.say("Please let me make a friend with you")
            rospy.sleep(3)
            self.record_signal.publish(True)
            self.detection_signal.publish(True)

        elif msg.data.find('find object') > -1:
            self.soundhandle.say("To initialize tracking, please select the object with mouse")
            rospy.sleep(5)    
            # self.xf.publish(True)
            self.camshift.publish(True)
            
        else:
            self.passtag = 0
            rospy.sleep(3)
        
        # rospy.sleep(5)
        while not self.workout and self.passtag:
            rospy.sleep(1)
        if self.passtag:
            self.soundhandle.say("I have finished the task. Hahaha. What can I do for you?")
            rospy.sleep(7)
        self.passtag = 1
        self.xf.publish(True)
        self.take_photo.publish(False)
        self.detection_signal.publish(False)
        self.record_signal.publish(False)
        self.camshift.publish(False)
        # rospy.spin()
        
        # while not rospy.is_shutdown():
        #     self.xf.publish(True)
        #     rospy.spin()

        #     self.rate.sleep()
        # Uncomment to play one of the built-in sounds
        # rospy.sleep(2)
        # self.soundhandle.play(5)

        # Uncomment to play a wave file
        # rospy.sleep(2)
        #self.soundhandle.playWave(self.wavepath + "/R2D2a.wav")

    def cleanup(self):
        self.soundhandle.stopAll()
        rospy.loginfo("Shutting down partybot node...")


if __name__ == "__main__":
    try:
        PartyBot(sys.path[0])
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Partybot node terminated.")
