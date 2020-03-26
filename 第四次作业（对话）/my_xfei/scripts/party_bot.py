#!/usr/bin/env python


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


class PartyBot:
    def __init__(self, script_path):
        rospy.init_node('partybot')

        rospy.on_shutdown(self.cleanup)

        self.rate = rospy.Rate(1)

        # Set the default TTS voice to use
        # self.voice = rospy.get_param("~voice", "voice_don_diphone")

        # Set the wave file path if used
        # self.wavepath = rospy.get_param(
        #     "~wavepath", script_path + "/../sounds")
        self.wavepath = "/home/ljc/catkin_ws/src/rc-home-edu-learn-ros/rchomeedu_apps/rchomeedu_partybot/sounds"

        # Create the sound client object
        self.soundhandle = SoundClient()
        # self.soundhandle = SoundClient(blocking=True)

        # Wait a moment to let the client connect to the sound_play server
        rospy.sleep(1)

        # Make sure any lingering sound_play processes are stopped.
        self.soundhandle.stopAll()

        # Announce that we are ready for input
        # self.soundhandle.playWave(self.wavepath + "/R2D2a.wav")
        # rospy.sleep(1)
        # self.soundhandle.say("Ready")

        rospy.loginfo("Ready, waiting for commands...")
        # self.soundhandle.say('Hello, I am PartyBot. What can I do for you?')
        # rospy.sleep(2)

        # Subscribe to the recognizer output and set the callback function
        rospy.Subscriber('voiceWords', String, self.talkback)
        self.xf = rospy.Publisher('voiceWakeup',Bool,queue_size=10)
        rospy.spin()

        # self.dance_arm = rospy.Publisher("dance_arm", String, queue_size=10)

        # self.take_photo = rospy.Publisher("take_photo", String, queue_size=10)

        # self.cmd_vel = rospy.Publisher(
        #     'cmd_vel_mux/input/navi', Twist, queue_size=10)


    def talkback(self, msg):
        # Print the recognized words on the screen
        # msg.data=msg.data.lower()
        rospy.loginfo(msg.data)
        self.xf.publish(False)
        # Speak the recognized words in the selected voice
        # self.soundhandle.say(msg.data, self.voice)
        # call('rosrun sound_play say.py "montreal"', shell=True)
        # rospy.sleep(1)

        if msg.data.find('introduce yourself') > -1:
            # self.soundhandle.playWave(self.wavepath + "/R2D2a.wav")
            # rospy.sleep(1)
            self.soundhandle.say(
                "I heard you want me to introduce myself. I am PartyBot. I am a party robot to serve you and have fun.")
            # rospy.sleep(10)
        elif msg.data.find('How old are you') > -1:
            # self.soundhandle.playWave(self.wavepath + "/R2D2a.wav")
            # rospy.sleep(1)
            self.soundhandle.say(
                "I heard you ask about my age. I am five years old.")
            # rospy.sleep(5)

        elif msg.data.find('story') > -1:
            # self.soundhandle.playWave(self.wavepath + "/R2D2a.wav")
            # rospy.sleep(1)
            self.soundhandle.say("OK. I will tell you a story about three pigs.")
            # rospy.sleep(5)
            
        elif msg.data.find('animal') > -1:
            # self.soundhandle.playWave(self.wavepath + "/R2D2a.wav")
            # rospy.sleep(1)
            self.soundhandle.say(
                "I heard you ask about what kind of animal I like. I like dogs more than cats.")

        elif msg.data.find('hometown') > -1:
            # self.soundhandle.playWave(self.wavepath + "/R2D2a.wav")
            # rospy.sleep(1)
            self.soundhandle.say(
                "I heard you ask about my hometown. I am from China.")
            # rospy.sleep(5)
        elif msg.data.find('can you do') > -1:
            # self.soundhandle.playWave(self.wavepath + "/R2D2a.wav")
            # rospy.sleep(1)
            self.soundhandle.say(
                "I heard you ask me what can I do? I am a home robot. I am good at singing and dancing. I tell funny jokes and I take great photos of people")
            # rospy.sleep(5)
        elif msg.data.find('joke') > -1:
            # self.soundhandle.playWave(self.wavepath + "/R2D2a.wav")
            # rospy.sleep(1)
            self.soundhandle.say(
                "You want to hear a joke? What is orange and sounds like a parrot? Erm, It is a carrot. Ha ha ha")
            # rospy.sleep(8)
        else:
            self.soundhandle.say(
                "Well,I know that"
            )
            #call('rosrun rchomeedu_vision take_photo.py', shell=True)
            # rospy.sleep(6)
            #call('rosrun image_view image_saver image:=/camera_top/rgb/image_raw _save_all_image:=false _filename_format:=foo.jpg __name:=image_saver', shell=True)
            #call('rosservice call /image_saver/save', shell=True)
            # rospy.sleep(6)
        # else: self.soundhandle.say("Sorry, I cannot hear you clearly. Please say again.")
        
        rospy.sleep(5)
        self.xf.publish(True)
        
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
