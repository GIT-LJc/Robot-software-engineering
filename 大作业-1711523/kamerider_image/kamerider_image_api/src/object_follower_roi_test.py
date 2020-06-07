#!/usr/bin/env python
# -*- coding: UTF-8 -*-
"""   
    Follow a target published on the /roi topic using depth from the depth image.
    说白了就是泛化的follow me 
    x 代表目标偏离中心的程度
    z 代表离目标的安全距离
    这个想法很简单,利用深度图中的深度信息,利用一个ROI的平均很自然的就可以得到一个平均深度
"""
from control.msg import findmymate
import rospy
from sensor_msgs.msg import Image, RegionOfInterest, CameraInfo , LaserScan
from geometry_msgs.msg import Twist
from math import copysign, isnan
import numpy as np
import math
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import thread
import roslib
from std_msgs.msg import String

class ObjectFollower():
    def __init__(self):
	self.speech_pub_to_control = rospy.Publisher('find_my_mate/control', findmymate, queue_size=1)

        rospy.init_node("roi_follower")
        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)

        self.control_rate = 10
        self.rate = rospy.get_param("~rate", self.control_rate)
        r = rospy.Rate(self.rate)
        self.scale_roi = rospy.get_param("~scale_roi", 0.9)


        self.max_linear_speed = rospy.get_param("~max_linear_speed", 0.5)
        self.min_linear_speed = rospy.get_param("~min_linear_speed", 0.001) 
        self.max_rotation_speed = rospy.get_param("~max_rotation_speed", 1.0)
        self.min_rotation_speed = rospy.get_param("~min_rotation_speed", 0.01)
        self.x_threshold = rospy.get_param("~x_threshold", 50)
        self.z_threshold = rospy.get_param("~z_threshold", 20)
        self.max_x = rospy.get_param("~max_x", 6000)
        self.min_x = rospy.get_param("~min_x", 800)
        self.goal_x = rospy.get_param("~goal_x",1300)
        self.unroi_rotate_speed = rospy.get_param("~unroi_rotate_speed",0.0)
        #-----------------------------下面就是简单的PID参数----------------------------
        # 角度控制参数
        self.z_kp = rospy.get_param("~z_kp", 1.0)
        self.z_ki = rospy.get_param("~z_ki",0.01)
        self.z_kd = rospy.get_param("~z_kd",0.01)        
        self.z_error = 0.0
        self.z_ierror = 0.0
        self.z_derror = 0.0
        # 线速度控制参数
        self.x_kp = rospy.get_param("~x_kp", 0.0005)#0.5
        self.x_ki = rospy.get_param("~x_ki",0.01)
        self.x_kd = rospy.get_param("~x_kd",0.01)
        self.x_error = 0.0
        self.x_ierror = 0.0
        self.x_derror = 0.0
        # 角速度衰减频率       
        self.slow_down_factor_rotate = rospy.get_param("~slow_down_factor_rotate",0.8)
        # 线速度衰减频率
        self.slow_down_factor_linear = rospy.get_param("~slow_down_factor_linear",0.8)        
        # -------------------------视觉信息处理-------------------------------
        # Initialize the global ROI 
        self.roi = RegionOfInterest()
        # Publisher to control the robot's movement
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=1)#size=5
        # Intialize the movement command
        self.move_cmd = Twist()
        # Get a lock for updating the self.move_cmd values
        self.lock = thread.allocate_lock()
        # We will get the image width and height from the camera_info topic
        self.image_width = 0
        self.image_height = 0
        # 深度图转ros中便于处理的图片,说白了就是消息处理
        self.cv_bridge = CvBridge()
        self.depth_array = None
        self.scan_array = None                
        # 判断ROI是否可见
        self.target_visible = False
        self.switch = False
        #----------------------------------收集camera相关信息-------------------------
        rospy.loginfo("Waiting for camera_info topic...")
        rospy.wait_for_message('/camera/rgb/camera_info', CameraInfo)
        # 得到图片的宽度和高度
        rospy.Subscriber('/camera/rgb/camera_info', CameraInfo, self.get_camera_info, queue_size=1)
        # 阻塞进程,直到我们真正收到图片信息
        while self.image_width == 0 or self.image_height == 0:
            rospy.sleep(1)
        # -------------------------------收集深度信息---------------------------------
        # 阻塞进程直到我们收到深度图(用于控制线速度)
        rospy.loginfo("Waiting for depth_image topic...")
        rospy.wait_for_message('/camera/depth/image_raw', Image)
        # ------------------------------收集ROI信息-----------------------------------
        # 接收深度图像
        self.depth_subscriber = rospy.Subscriber("/camera/depth/image_raw", Image, self.convert_depth_image, queue_size=1)
        # 接收ROI同时根据ＲＯＩ控制机器人
        rospy.Subscriber('roi', RegionOfInterest, self.set_cmd_vel, queue_size=1)
        #rospy.Subscriber('position', RegionOfInterest, self.set_cmd_vel, queue_size=1)
        # ------------------------------收集激光信息------------------------------------
        # 但是激光信息如何与ROI对上? 激光信息没有办法和ROI对上
        rospy.loginfo("Waiting for scan topic...")
        rospy.wait_for_message('/scan', LaserScan)
        self.depth_subscriber = rospy.Subscriber("/scan", LaserScan, self.collect_scan, queue_size=1)
        # ------------------------------等待ROI信息-------------------------------------        
        # Subscribe to the ROI topic and set the callback to update the robot's motion
        #rospy.Subscriber('roi', RegionOfInterest, self.set_cmd_vel, queue_size=1)
        # Wait until we have an ROI to follow
        rospy.loginfo("Waiting for an ROI to track...")
        rospy.wait_for_message('roi', RegionOfInterest)
        rospy.loginfo("ROI messages detected. Starting follower...")
        #------------------------------开始跟踪------------------
        # 保证每次可以成功发送信息
        # Begin the tracking loop

    # -----------------------------关键的根据ROI和深度图处理视觉信息-------------------------                    
    def set_cmd_vel(self, msg):
        # Acquire a lock while we're setting the robot speeds
        self.lock.acquire()
        try:
            # If the ROI has a width or height of 0, we have lost the target
            if msg.width == 0 or msg.height == 0:
	        if self.switch==False:
                    self.target_visible = False
                    # 保证找不到图的时候我可以自动旋转去找,这是一种比较简单的方法,效果还行
                    self.move_cmd.linear.x = 0
                    self.move_cmd.angular.z = self.unroi_rotate_speed
                    rospy.loginfo("NO ROI Has been found!")
		    self.cmd_vel_pub.publish(self.move_cmd)
		    print(self.move_cmd)
		    print("---------------------------------------------------------------------------")
                    return
            else:
                self.target_visible = True
	    print(msg)
	    self.switch=True
            print("-------------------Rotation Control---------------------")            
            # 把当前的ROI记录下来,其实我比较担心的是深度图的时间戳和ROI无法对齐,但是讲道理深度图应该是可以实时得到
            self.roi = msg

	    self.move_cmd.angular.z = 0
            print("-------------------Linear Control---------------------")
            '''----------------目前采用深度图的方法,后期可以采用点云,激光的方法进行解决深度问题--------------'''
            # 但是我感觉根据深度图来看,才有道理吧,根据深度图来看,我才能知道ROI的具体范围和对应大小
            # ---------------计算深度信息,上面主要是旋转,利用一个简单的P控制----------------------
            # TODO:用来计算ROI中部分深度的平均值,直接用激光反而麻烦,当然深度如果探测不到,应该取最大?
            n_x = sum_x = mean_x = 0
            # 缩小ROI降低噪声相当于取ROI一部分的深度，这样写比较方便。
            scaled_width = int(self.roi.width * self.scale_roi)
            scaled_height = int(self.roi.height * self.scale_roi)
            # 说白了就是得到新的划窗的范围
            min_x = int(self.roi.x_offset + self.roi.width * (1.0 - self.scale_roi) / 2.0)
            max_x = min_x + scaled_width
            min_y = int(self.roi.y_offset + self.roi.height * (1.0 - self.scale_roi) / 2.0)
            max_y = min_y + scaled_height
            #--------------------------说白了在缩小的ROI图的深度方向进行计算对应的平均深度,当然也可以取最大深度,但是这个一些信息损失--------------------
            # 计算roi的平均深度
            real_mean = 0.0
            for x in range(min_x, max_x):
                for y in range(min_y, max_y):
                    try:
                        # 得到ROI中的深度数据(meters)
                        x_ = self.depth_array[y, x]

                        # TODO:检查是否有NAN数
                        # 果然就是太远了
                        if isnan(x_):
                            x_ = self.max_x
                    except:
                        # 说白了为0就可以阻塞速度,只要不炸掉,了不起你不动就好了
                        x_ = 0                      
           
		    if x_==0:
			n_x-=1
                    sum_x = sum_x + x_
                    n_x += 1
            print("1 ----------------GET AVERAGE DEPTH--------------------")
            try:
		print(sum_x,n_x)
                print("mean depth %f mm"%(sum_x/n_x))
            except:
                print("No n_x")
            # --------------------------------停止得到对应的深度信息--------------------------------------
            # 默认停止机器人
            linear_x = 0
            print("2 ----------------GET SPEED --------------------")
            # 如果存在深度数据
            if n_x:
                mean_x = float(sum_x) / float(n_x)                                                             
                # TODO:不要小于最小深度,但是这样不是错的么。。因为假设都到了最小了,你他妈不就炸掉了
                mean_x = max(self.min_x, mean_x)                                            
                # 说白了如果上面min更大,他不会发速度
                # TODO:如何避免撞上去?
                if mean_x > self.min_x:
                    # 如果平均深度距离goal差距很大,那么就调整
                    # 这里就是一个正向的,没啥奇怪的
                    if (abs(mean_x - self.goal_x) > self.x_threshold):
                        speed = (mean_x - self.goal_x) * self.x_kp
                        linear_x = copysign(np.clip(math.fabs(speed),self.min_linear_speed,self.max_linear_speed), speed)
                        print("Given  speed : %f m/s"%speed)
                        print("Real speed : %f m/s"%linear_x)
            if linear_x == 0:
               # 如果给定很小那么就衰减
                self.move_cmd.linear.x *= self.slow_down_factor_linear
            else:
                # 否则就不衰减
                self.move_cmd.linear.x = linear_x
            self.move_cmd.angular.z = 0
            print("finish%f"%self.move_cmd.linear.x)
	

 
	    if abs(self.move_cmd.linear.x)<0.03:
                task = findmymate()
                task.NowTask   = task.talktake
                task.NextTask = task.EnterGate
                task.FinishState = True
                task.NeedHelp = False
                self.speech_pub_to_control.publish(task)

 
	    self.switch=False
	    self.cmd_vel_pub.publish(self.move_cmd)
	    print(self.move_cmd)
        finally:
            # Release the lock
            self.lock.release()

    def convert_depth_image(self, ros_image):
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            # Convert the depth image using the default passthrough encoding
            depth_image = self.cv_bridge.imgmsg_to_cv2(ros_image, "passthrough")
            # print(depth_image) 
        except CvBridgeError, e:
            print e

        # Convert the depth image to a Numpy array
        self.depth_array = np.array(depth_image, dtype=np.float32)

        

    def get_camera_info(self, msg):
        self.image_width = msg.width
        self.image_height = msg.height
    def collect_scan(self,msg):
        return 
        # print("scan")
        # print(msg.ranges)
        # self.scan_array = np.array(msg,dtype=np.float32)
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        # Unregister the subscriber to stop cmd_vel publishing
        self.depth_subscriber.unregister()
        rospy.sleep(1)
        # Send an emtpy Twist message to stop the robot
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)      

if __name__ == '__main__':
    try:
        ObjectFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Object follower node terminated.")

