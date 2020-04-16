#!/usr/bin/env python

"""

    RoboCup@Home Education | oc@robocupathomeedu.org
    navi.py - enable turtlebot to navigate to predefined waypoint location

"""

import rospy

import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Bool
from sound_play.libsoundplay import SoundClient
from ch3_pkg.msg import Num
original = 0


class NavToPoint:

    def __init__(self):
        # rospy.init_node('navigation')
        rospy.on_shutdown(self.cleanup)
        self.locat = rospy.Subscriber('/location', Num, self.loc)        
	# Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")

        # Wait for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(120))
        rospy.loginfo("Connected to move base server")

        # A variable to hold the initial pose of the robot to be set by the user in RViz
        initial_pose = PoseWithCovarianceStamped()
        self.init = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.update_initial_pose)
        # self.locat = rospy.Subscriber('/location', Num, self.loc)
        self.outcome = rospy.Publisher("finish",Bool,queue_size=1)
		# rospy.Subscriber('location', Pose, self.loc)

        self.soundhandle = SoundClient()
        rospy.sleep(1)
        # Make sure any lingering sound_play processes are stopped.
        self.soundhandle.stopAll()
        self.start = 1

	# Get the initial pose from the user
        # rospy.sleep(15)
        # self.soundhandle.say("Click the 2D Pose Estimate button in RViz to set the robot's initial pose please.")
        rospy.loginfo("*** Click the 2D Pose Estimate button in RViz to set the robot's initial pose...")
        rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)
        
        # Make sure we have the initial pose
        while initial_pose.header.stamp == "":
        	rospy.sleep(1)
            
        rospy.loginfo("Ready to go")
        rospy.sleep(1)
        rospy.spin()



    def loc(self, msg):
		if self.start:
			self.start = 0
			self.soundhandle.say("Click the 2D Pose Estimate button in RViz to set the robot's initial pose please.")
			rospy.sleep(5)
		    

		locations = dict()

		# Location A
		A_x = msg.A_x
		A_y = msg.A_y
		A_theta = 0.0
		print("POSTION A--------------------(x,y)",A_x,A_y)
		quaternion = quaternion_from_euler(0.0, 0.0, A_theta)
		locations['A'] = Pose(Point(A_x, A_y, 0.000), Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]))
		# locations['A'] = msg.data


		self.goal = MoveBaseGoal()
		rospy.loginfo("Starting navigation test")
		rospy.loginfo("Going to point A")
		tag = 0


		while not rospy.is_shutdown() and tag==0 :
			self.goal.target_pose.header.frame_id = 'map'
			self.goal.target_pose.header.stamp = rospy.Time.now()

			# Robot will go to point A
			# if start == 1:
			# rospy.loginfo("Going to point A")
			rospy.sleep(2)
			self.goal.target_pose.pose = locations['A']
			print(locations['A'],"----------------------------------------------")
			self.move_base.send_goal(self.goal)
			waiting = self.move_base.wait_for_result(rospy.Duration(300))
			if waiting == 1:
				tag = tag + 1
				rospy.loginfo("Reached point A")
				rospy.sleep(2)
				if tag == 1:
					self.outcome.publish(True)
					# rospy.loginfo("Ready to go back")
					# rospy.sleep(2)
					# global start
					# start = 0

		# # After reached point A, robot will go back to initial position
		# elif start == 0:
		# 	rospy.loginfo("Going back home")
		# 	rospy.sleep(2)
		# 	self.goal.target_pose.pose = self.origin
		# 	self.move_base.send_goal(self.goal)
		# 	waiting = self.move_base.wait_for_result(rospy.Duration(300))
		# 	if waiting == 1:
		# 		rospy.loginfo("Reached home")
		# 		rospy.sleep(2)
		# 		global start
		# 		start = 2

			rospy.Rate(5).sleep()


    def update_initial_pose(self, initial_pose):
        self.initial_pose = initial_pose
        # if original == 0:
        self.origin = self.initial_pose.pose.pose
			# global original
			# original = 1

    def cleanup(self):
        rospy.loginfo("Shutting down navigation	....")
        self.move_base.cancel_goal()


if __name__=="__main__":
    rospy.init_node('navi_point')
    try:
        NavToPoint()
        rospy.spin()
    except:
        pass

