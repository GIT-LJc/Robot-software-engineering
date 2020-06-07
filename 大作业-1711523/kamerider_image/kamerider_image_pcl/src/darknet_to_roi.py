#!/usr/bin/env python
# -*- coding: utf-8 -*

from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import RegionOfInterest

import rospy
import roslib


class DarktoRoi:
    def __init__(self):
        self.target = rospy.get_param('~target_name',"person")
        # 发布器
        self.yolo_sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.yoloCallback)
        self.roi_pub = rospy.Publisher("roi", RegionOfInterest)
    def yoloCallback(self, msg):
        roi = RegionOfInterest()
        for data in msg.bounding_boxes:
            if data.Class == self.target:
                roi.x_offset = data.xmin
                roi.y_offset = data.ymin
                roi.width = data.xmax -data.xmin
                roi.height = data.ymax - data.ymin
                self.roi_pub.publish(roi)
        return 0
if __name__ == "__main__":
    rospy.init_node('darknet_to_roi', anonymous=True)
    DtR = DarktoRoi()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
