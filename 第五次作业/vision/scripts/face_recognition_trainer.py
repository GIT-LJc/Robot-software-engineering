#! /usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2017, Yuki Furuta.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Kei Okada nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


from __future__ import print_function

try:
    input = raw_input
except:
    pass

import rospy
import message_filters
from sensor_msgs.msg import Image
from opencv_apps.msg import FaceArrayStamped
from opencv_apps.srv import FaceRecognitionTrain, FaceRecognitionTrainRequest
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import String
from std_msgs.msg import Bool



class FaceRecognitionTrainer(object):
    def __init__(self):
        self.queue_size = rospy.get_param("~queue_size", 100)
        
        self.img_sub = message_filters.Subscriber("image", Image)
        self.face_sub = message_filters.Subscriber("faces", FaceArrayStamped)
        
        self.req = FaceRecognitionTrainRequest()
        self.label = ""
        self.ok = False

        self.sync = message_filters.TimeSynchronizer([self.img_sub, self.face_sub],
                                                     self.queue_size)
        self.sync.registerCallback(self.callback)

        self.record_signal = rospy.Subscriber("record_signal", Bool, self.run)
        self.record_result = rospy.Publisher("record_result", String, queue_size=10)
        self.start = False
        self.soundhandle = SoundClient()
        rospy.sleep(1)
        self.soundhandle.stopAll()
        rospy.loginfo("Ready, waiting for commands...")
        # self.soundhandle.say('Hello, I am recorder. What can I do for you?')
        rospy.sleep(5)
        self.name = ""
        rospy.Subscriber('/voiceWords', String, self.namecall)
        self.xf = rospy.Publisher('voiceWakeup',Bool,queue_size=10)
        self.finish = rospy.Publisher("/finish",Bool,queue_size=10)
        # self.xf.publish(True)
        rospy.spin()
    
    # def startnow(self,msg):
    #     self.start = msg.data

    def namecall(self, msg):
        self.label = msg.data



    def callback(self, img, faces):
        if len(faces.faces) <= 0:
            return
        if self.ok:
            faces.faces.sort(key=lambda f: f.face.width * f.face.height)
            self.req.images.append(img)
            self.req.rects.append(faces.faces[0].face)
            self.req.labels.append(self.label)
            self.ok = False

    def run(self,msg):
        self.start = msg.data
        print("start recording: ",self.start)
        if self.start:
            print(1)
            rospy.wait_for_service("train")
            print(2)
            train = rospy.ServiceProxy("train", FaceRecognitionTrain)
            self.xf.publish(False)
            self.soundhandle.say('Hello, I am recorder. Nice to meet you.')
            rospy.sleep(5)
            self.soundhandle.say("Please tell me your name: ")
            rospy.sleep(2)
            self.xf.publish(True)
            # self.label = input("Please input your name and press Enter: ")
            rospy.sleep(7)
            if len(self.label) <= 0:
                self.xf.publish(False)
                self.soundhandle.say("Please tell me your name, thanks")
                # self.label = input("Please input your name and press Enter: ")
                rospy.sleep(3)
                self.xf.publish(True)
                rospy.sleep(4)
            print("name_label-------------:",self.label)
            self.soundhandle.say("Please stand at the center of the camera and I will record your face")
            # input("Please stand at the center of the camera and press Enter: ")
            rospy.sleep(3)
            self.xf.publish(True)
            rospy.sleep(5)
            # print("yes_label-------------:",self.label)
            while self.label.find("Yes")<0:
                rospy.sleep(1)
            while self.label.find("Yes")>-1:
                print("yes_label-------------:",self.label)
                self.xf.publish(False)
                self.soundhandle.say("taking picture")
                rospy.sleep(3)
                print("taking picture...")
                # rospy.sleep(3)
                self.soundhandle.say("One more picture?")
                rospy.sleep(4)
                self.xf.publish(True)
                rospy.sleep(1)
                while not self.label:
                    rospy.sleep(1)
                if self.label.find("No")>-1:
                # if input("One more picture? [y/n]: ") not in ["", "y", "Y"]:
                    break
            print("sending to trainer...")
            self.soundhandle.say("sending to trainer")
            
            res = train(self.req)
            if res.ok:
                self.xf.publish(False)
                print("OK. Trained successfully!")
                self.soundhandle.say('OK. Trained successfully!')
                rospy.sleep(3)
                self.record_result.publish("Nice to meet you!" + str(self.req.labels))
                rospy.sleep(3)
                self.finish.publish(True)
            else:
                print("NG. Error: %s" % res.error)

if __name__ == '__main__':
    rospy.init_node("face_recognition_trainer")
    t = FaceRecognitionTrainer()
    # t.run()
    rospy.spin()
