#!/usr/bin/env python

'''
Copyright (c) 2016, Nadya Ampilogova
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

# Script for simulation
# Launch gazebo world prior to run this script

from __future__ import print_function
import sys
import roslib
import rospy
import cv2
from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time
from sound_play.libsoundplay import SoundClient
from opencv_apps.msg import FaceArrayStamped

class TakePhoto:
    def __init__(self):
        rospy.init_node('take_photo_sub')

        rospy.on_shutdown(self.cleanup)

        self.bridge = CvBridge()
        self.image_received = False

        # Connect image topic
        #img_topic = "/camera/rgb/image_raw"
        #img_topic = "/camera_top/rgb/image_raw"
        img_topic = "/usb_cam/image_raw"
        self.image_sub = rospy.Subscriber(img_topic, Image, self.callback)
        self.finish = rospy.Publisher("/finish",Bool,queue_size=10)
        self.rate = rospy.Rate(1)
        rospy.Subscriber('/take_photo', Bool, self.take_photo)
        rospy.Subscriber('/voiceWords', String, self.pause)
        self.xf = rospy.Publisher('voiceWakeup',Bool,queue_size=10)
        self.second = 0

        # msg_pub_ = advertise<opencv_apps::FaceArrayStamped>(*pnh_, "faces", 1)
        rospy.Subscriber('face_detection/faces', FaceArrayStamped ,self.face)
        self.faces = None
        self.pos = []

        self.soundhandler = SoundClient()
        rospy.sleep(5)
        # self.soundhandler.stopAll()
        rospy.loginfo("take photos hahaha")
        # self.xf.publish(True)
        # self.soundhandler.say('take photos. ha ha ha.')
        rospy.spin()

    def face(self, faces):
        self.pos = []
        if len(faces.faces) <= 0:
            return
        else:
            self.faces = faces
            self.faces.faces.sort(key=lambda f: f.face.x)
            for f in self.faces.faces:
                self.pos.append([f.face.x,f.face.y])
            # self.req.images.append(img)
            # self.req.rects.append(faces.faces[0].face)
            # self.req.labels.append(self.label)
            # self.ok = False
    
    def pause(self, msg):
        if msg.data.find('three')>-1:
            self.second = 3
        elif msg.data.find('five')>-1:
            self.second = 5
        elif msg.data.find('ten')>-1:
            self.second = 10
        else:
            self.second = None

    def callback(self, data):

        # Convert image to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.image_received = True
        self.image = cv_image

    def take_picture(self, img_title):
        if self.image_received:
            # Save an image
            cv2.imwrite(img_title, self.image)
            return True
        else:
            return False

    def take_photo(self, msg):
        #print msg.data
        if msg.data == True:
            # Take a photo
            # Use '_image_title' parameter from command line
            # Default value is 'photo.jpg'
            #img_title = rospy.get_param('~image_title', 'photo.jpg')
            timestr = time.strftime("%Y%m%d-%H%M%S-")
            self.xf.publish(False)
            img_title = "/home/ljc/" + timestr + "photo.jpg"
            self.soundhandler.say('How long?')
            rospy.sleep(3)
            self.xf.publish(True)
            rospy.sleep(6)
            
            while True:
                while not self.second:
                    rospy.sleep(1)
                self.soundhandler.say("I will take a photo after %s seconds" %self.second)
                if self.second:
                    rospy.sleep(self.second)
                    self.xf.publish(False)

                    tag = 1
                    while not self.pos:
                        rospy.sleep(1)
                    
                    if self.pos:
                        print("positiion--face-------",self.pos)

                    while self.pos and tag:
                        for p in self.pos:
                            if p[0]<280 :
                                self.soundhandler.say('Please move to the left side,thanks.')
                                rospy.sleep(3)
                                tag=1
                                break
                            elif p[0]>320:
                                self.soundhandler.say('Please move to the right side,thanks.')
                                rospy.sleep(3)
                                tag=1
                                break
                            elif p[1]<180:
                                self.soundhandler.say('Please move down,thanks.')
                                rospy.sleep(3)
                                tag=1
                                break
                            elif p[1]>280:
                                self.soundhandler.say('Please move up,thanks.')
                                rospy.sleep(3)
                                tag=1
                                break
                            else:
                                tag = 0
                    
                    self.soundhandler.say(' Ok, get ready. One, two, three, say cheese')
                    rospy.sleep(5)
                    if self.take_picture(img_title):
                        rospy.loginfo("Saved image " + img_title)
                        self.soundhandler.say("I have taken your photos")
                        rospy.sleep(3)
                        self.finish.publish(True)
                        break
                    else:
                        rospy.loginfo("No images received")
                else:
                    rospy.loginfo("wait...")

    def cleanup(self):
        self.soundhandler.stopAll()
        rospy.loginfo("Shutting down take_photo node...")
 
if __name__ == '__main__':

    # Initialize
    rospy.init_node('take_photo_sub', anonymous=False)
    TakePhoto()
 
    rospy.spin()
    # Sleep to give the last log messages time to be sent
    #rospy.sleep(1)
