#!/usr/bin/env python
# -*- coding: utf-8 -*

# 使用百度API进行关键点视觉处理
# 实际上可以使用openpose，使用同方法替代，但是其算力要求过高。
# 现在是检测肘，腕，肩三个位置判断。
# 发布消息：
#   视野中目标的位置
#   带有目标的图像


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

# ROS
import rospy
import roslib
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int8
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import RegionOfInterest
from geometry_msgs.msg import Pose

class BodyCheck:
    def __init__(self):
        self.time = time.time()
        APP_ID = '18889374'
        API_KEY = 'pUNweNaSK4rWz57vGs9KpuW1'
        SECRET_KEY = 'ru5LqWM0lrcVYBh9cjd32fy951nagqcA'
        self.image_type = "BASE64"
        self.client_face = AipFace(APP_ID, API_KEY, SECRET_KEY)
        self.client_body = AipBodyAnalysis(APP_ID, API_KEY, SECRET_KEY)
        self.client_body.setConnectionTimeoutInMillis(2000)
        self.client_body.setSocketTimeoutInMillis(2000)
        self.client_face.setConnectionTimeoutInMillis(2000)
        self.client_face.setSocketTimeoutInMillis(2000)
        self.bridge = CvBridge()
	self.ispub=False
        ##############人类数据
        self.filepath = "/home/qian/catkin_ws/src/fmm_src/kamerider_image/kamerider_image_api/imgfile/"
        ##############跟踪数据
        self.roi = None
        self.depth_array = None
        self.target_pos = Pose()

        ##############话题名称
        # 接收器与发布器
        self.check_gender = rospy.get_param('~check_gender',                  'False')

        if(type(self.check_gender) == type('text')):
            self.check_gender = False

        ##############发布器
        self.img_pub = rospy.Publisher("/image/test", Image,queue_size=1)
        self.roi_pub = rospy.Publisher("roi", RegionOfInterest,queue_size=1)
        self.img_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.imgCallback,queue_size=1)
        self.word_pub = rospy.Publisher("/xfwords", String,queue_size=1)
        self.face_pub = rospy.Publisher("/start_recognize_faces", String,queue_size=1)

	
        print("============================================================")

    # 人体数据转换
    def msgtobody(self, image_msg, file_name='image_body.png'):
        # 转为base64存储
        cv2.imwrite(self.filepath+file_name, image_msg)
        with open(self.filepath+file_name, 'rb') as fp:
            return fp.read()

    # 人脸数据转换
    def msgtoface(self,image_msg, file_name='image_faces.png'):
        cv2.imwrite(self.filepath+file_name, image_msg)
        with open(self.filepath+file_name, 'rb') as fp:
            data = base64.b64encode(fp.read())
            # python2.7
            data = str(data).encode('utf-8')
            return data

    # 挥手检测，返回一个挥手的人的方框xxyy数据
    def detectWave(self, image, gender=False):
        print("CHECK")
        data = self.msgtobody(image, "image_body.png")
        # ----------挥手检测----------
        result = self.client_body.bodyAnalysis(data)
        wave = []
        loaction = []
        point_t = []
        # 存在人
        if result['person_num'] > 0:
            id_ = -1
            # 对每个人进行检查
            for info in result['person_info']:
                id_+=1
                keypoint = info['body_parts']
                # 腕高
                if keypoint['right_elbow']['y'] > keypoint['right_wrist']['y']:
                    # 腕在外侧
                    if keypoint['right_wrist']['x'] < keypoint['right_shoulder']['x']:
                        wave.append(id_)
                        loc = []
                        loc.append(int(info['location']['left']))
                        loc.append(int(info['location']['left'] + info['location']['width']))
                        loc.append(int(info['location']['top']))
                        loc.append(int(info['location']['top']  + info['location']['height']))
                        loaction.append(copy.deepcopy(loc))
                # 腕高
                elif keypoint['left_elbow']['y'] > keypoint['left_wrist']['y']:
                    # 腕在外侧
                    if keypoint['left_wrist']['x'] > keypoint['left_shoulder']['x']:
                        wave.append(id_)
                        loc = []
                        loc.append(int(info['location']['left']))
                        loc.append(int(info['location']['left'] + info['location']['width']))
                        loc.append(int(info['location']['top']))
                        loc.append(int(info['location']['top']  + info['location']['height']))
                        loaction.append(copy.deepcopy(loc))

            # 存在挥手
            if len(loaction) > 0:
                # 女性检测
                # ----------性别检测----------
                if gender:
                    options = {}
                    options["type"] = "gender"
                    # 保证存在挥手
                    for locate in loaction:
                        img = image[locate[2]:locate[3],
                                    locate[0]:locate[1]]
                        img = self.msgtobody(img, "image_face.png")
                        result = self.client_body.bodyAttr(img, options)
                        try:
                            result['person_info'][0]['attributes']['gender'] == "女性"
                        except:
                            continue
                        # 女性则直接返回女性位置
                        if result['person_info'][0]['attributes']['gender'] =="女性":
                            loc = []
                            loc.append(locate[0])
                            loc.append(locate[1])
                            loc.append(locate[2])
                            loc.append(locate[3])
                            return locate
                # 随机返回一个人
                locate = loaction[0]
                loc = []
                loc.append(locate[0])
                loc.append(locate[1])
                loc.append(locate[2])
                loc.append(locate[3])
                return locate
        return None

    # 照片的回调函数，发布挥手人的位置
    def imgCallback(self, image):
	num=0
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            print(e)
        # 如果存在人
        roi = RegionOfInterest()
        try:
            position = self.detectWave(cv_image,self.check_gender)
            cv2.rectangle(cv_image, (position[1],position[3]),(position[0],position[2]),(0,0,255))
            roi.x_offset = position[0]
            roi.y_offset = position[2]
            roi.width = position[1] - position[0]
            roi.height = position[3] - position[2]
            self.roi = roi
            self.roi_pub.publish(roi)
	    if self.ispub==False:
	        #stringq="I can tell one wave. Now I will recognize faces. "
                #self.word_pub.publish(stringq)
	        stringq="ok "
                self.face_pub.publish(stringq)
		self.ispub=True
            rospy.loginfo("One Wave!")
	    num=1
        except:
            self.roi = roi
	if num==0:
            self.roi_pub.publish(roi)
        cv_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        self.img_pub.publish(cv_image)
    
if __name__ == "__main__":
    rospy.init_node('wave_detect', anonymous=True)
    body = BodyCheck()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("CLOSE WAVE CHECK")


