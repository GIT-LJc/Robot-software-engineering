#!/usr/bin/env python
# -*- coding: utf-8 -*

# 使用百度API进行挥手+特征识别
# 暂时没有替代方案
# 现在是检测肘，腕，肩三个位置判断挥手。然后使用API识别人的特征并发送出去
# 发布消息：
#   视野中目标的位置
#   带有目标的图像
#   特征信息


# 视觉处理
import cv2
from aip import AipFace
from aip import AipBodyAnalysis

# 基本类型
import numpy as np
import time
import base64
import matplotlib.pyplot as plt
import base64
import copy
import sys
reload(sys)
sys.setdefaultencoding('utf-8')


# ROS
import rospy
import roslib
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int8
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import RegionOfInterest
from geometry_msgs.msg import Pose
from speech.msg import Description



class BodyCheck:
    def __init__(self):
        self.time = time.time()
        APP_ID = '18889374'
        API_KEY = 'pUNweNaSK4rWz57vGs9KpuW1'
        SECRET_KEY = 'ru5LqWM0lrcVYBh9cjd32fy951nagqcA'
        self.imageType = "BASE64"
        self.client_face = AipFace(APP_ID, API_KEY, SECRET_KEY)
        self.client_body = AipBodyAnalysis(APP_ID, API_KEY, SECRET_KEY)

        self.client_body.setConnectionTimeoutInMillis(2000)
        self.client_body.setSocketTimeoutInMillis(2000)
        self.client_face.setConnectionTimeoutInMillis(2000)
        self.client_face.setSocketTimeoutInMillis(2000)
        
        self.bridge = CvBridge()
        ##############人类数据
        self.option_face = {}
        self.option_body = {}
        self.option_face["face_field"] = "age,gender,glasses,race"
        self.option_face["max_face_num"] = 1
        self.option_body["type"] = "upper_wear,upper_color"

        ##############跟踪数据
        self.roi = RegionOfInterest()

        ##############话题名称
        self.filepath            = rospy.get_param('~image_temp_save_path','')

        ##############发布器
        self.fet_pub = rospy.Publisher("/image/feature", Description)

        self.img_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.imgCallback,queue_size=1)
        self.roi_sub = rospy.Subscriber("roi", RegionOfInterest, self.roiCallback,queue_size=1)
        print("============================================================")
    ######################################################################3
    # 人体数据转换
    def msgtobody(self, image_msg, file_name='image_body.png'):
        # 转为base64存储
        cv2.imwrite(self.filepath+file_name, image_msg)
        with open(self.filepath+file_name, 'rb') as fp:
            return fp.read()
    ######################################################################3
    # 人脸数据转换
    def msgtoface(self,image_msg, file_name='image_faces.png'):
        cv2.imwrite(self.filepath+file_name, image_msg)
        with open(self.filepath+file_name, 'rb') as fp:
            data = base64.b64encode(fp.read())
            # python2.7
            data = str(data).encode('utf-8')
            return data
    ######################################################################3
    # 挥手检测，返回一个挥手的人的方框xxyy数据
    def roiCallback(self, msg):
        self.roi.x_offset = msg.x_offset
        self.roi.y_offset = msg.y_offset
        self.roi.width = msg.width
        self.roi.height = msg.height
    ######################################################################3
    # 检测人的特征
    def detedtFeature(self, image):
        position = [self.roi.x_offset,
                    self.roi.x_offset + self.roi.width,
                    self.roi.y_offset,
                    self.roi.y_offset + self.roi.height
                    ]
        if (int(position[0])==0 and int(position[1])==0 and int(position[2])==0 and int(position[3])==0):
            return None
        msg = Description()
        msg.hair_style = "unknown"
        msg.pose = "unknown"
        # 特征检测——人脸和人体
        img_body = image[position[2]:position[3],position[0]:position[1]]
        face = self.msgtoface(img_body)
        result1 = self.client_face.detect(face, self.imageType, self.option_face)
        try:
            data = result1["result"]["face_list"][0]
        except:
            print("Can not have face info")
            return 0
        # 性别 + 眼睛 + 肤色 + 年龄
        msg.gender = data["gender"]["type"]
        msg.glasses = data["glasses"]["type"]
        msg.skin_color = data["race"]["type"]
        msg.age = str(data["age"])

        # 颜色 + 服装
        body = self.msgtobody(img_body)
        result2 = self.client_body.bodyAttr(body, self.option_body)
        try:
            data = result2["person_info"][0]["attributes"]
        except:
            print("Can not have person info")
            return 0
        print("Check Feature")
        # 红、橙、黄、绿、蓝、紫、粉、黑、白、灰、棕
        color =  data["upper_color"]["name"]
        if color == "红":
            msg.clothes_color = "red"
        elif color == "橙":
            msg.clothes_color = "orange"
        elif color == "黄":
            msg.clothes_color = "yellow"
        elif color == "绿":
            msg.clothes_color = "green"
        elif color == "蓝":
            msg.clothes_color = "blue"
        elif color == "紫":
            msg.clothes_color = "purple"
        elif color == "粉":
            msg.clothes_color = "pink"
        elif color == "黑":
            msg.clothes_color = "black"
        elif color == "白":
            msg.clothes_color = "white"
        elif color == "灰":
            msg.clothes_color = "gray"
        else:
            msg.clothes_color = "brown"
        type_ = data["upper_wear"]["name"]
        if type_ == "长袖":
            msg.clothes = "Coat"
        else:
            msg.clothes = "Short"
        self.fet_pub.publish(msg)
    ######################################################################3
    # 照片的回调函数，发布人的特征
    def imgCallback(self, image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
            self.detedtFeature(cv_image)

           
        except CvBridgeError as e:
            print(e)





    
if __name__ == "__main__":
    rospy.init_node('feature_detect', anonymous=True)
    body = BodyCheck()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("CLOSE WAVE CHECK")
