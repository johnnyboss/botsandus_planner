#!/usr/bin/env python

NAME = 'bostandus_planner_test'

import sys, unittest, time
import math
import rospy, rostest
from geometry_msgs.msg import PoseWithCovarianceStamped
import actionlib

import tf
from tf import TransformListener
from move_base_msgs.msg import *
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
from std_msgs.msg import *
class Testbostandus_planner(unittest.TestCase):
	def __init__(self, *args):
		super(Testbostandus_planner, self).__init__(*args)
   		self.success = False

	def test_goal_sender(self):
		rospy.init_node('planner_test')
		self.pose_seq = list()
		self.goal_cnt = 0
		self.tf_listener = TransformListener()
		self.create_test_points()
		self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
		rospy.loginfo("Waiting for move_base action server... aaaaa")
		wait = self.client.wait_for_server(rospy.Duration(50.0))
		if not wait:
			rospy.logerr("Action server not available!")
			rospy.signal_shutdown("Action server not available!")
			return
		rospy.loginfo("Connected to move base server")
		rospy.loginfo("Starting goals achievements ...")
		self.movebase_client()
		
		timeout_t = time.time() + 120.0 #maybe 2 min for all goals

		while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
			
			time.sleep(0.1)


		
		self.assert_(self.success)

	
	def active_cb(self):
		rospy.loginfo("Goal pose "+str(self.goal_cnt+1)+" is now being processed by the Action Server...")

	def feedback_cb(self, feedback):
		#rospy.loginfo("Feedback for goal pose "+str(self.goal_cnt+1)+" received")
		a = 1

	def done_cb(self, status, result):
		self.goal_cnt += 1
		if status == 2:
			rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request after it started executing, completed execution!")

		if status == 3:
			rospy.loginfo("Goal pose "+str(self.goal_cnt)+" reached") 
			if self.goal_cnt< len(self.pose_seq):
				now = rospy.Time.now()
				self.tf_listener.waitForTransform("map","base_link",now,rospy.Duration(10.0))
				(trans,rot) = self.tf_listener.lookupTransform("map","base_link",now)
				rot = tf.transformations.euler_from_quaternion(rot)
				if math.fabs(math.degrees(rot[2]) - self.angles[self.goal_cnt-1]) > 10:
					rospy.signal_shutdown("Fail desired orientation")
				next_goal = MoveBaseGoal()
				next_goal.target_pose.header.frame_id = "map"
				next_goal.target_pose.header.stamp = rospy.Time.now()
				next_goal.target_pose.pose = self.pose_seq[self.goal_cnt]
				rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
				rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
				self.client.send_goal(next_goal, self.done_cb, self.active_cb, self.feedback_cb) 
			else:
				rospy.loginfo("Final goal pose reached!")
				self.success = True
				rospy.signal_shutdown("Final goal pose reached!")
				
				

		if status == 4:
			rospy.loginfo("Goal pose "+str(self.goal_cnt)+" was aborted by the Action Server")
			rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" aborted, shutting down!")
			return

		if status == 5:
			rospy.loginfo("Goal pose "+str(self.goal_cnt)+" has been rejected by the Action Server")
			rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" rejected, shutting down!")
			return

		if status == 8:
			rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request before it started executing, successfully cancelled!")

	def movebase_client(self):
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = "map"
		goal.target_pose.header.stamp = rospy.Time.now() 
		goal.target_pose.pose = self.pose_seq[self.goal_cnt]
		rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
		rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
		self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
		

	def create_test_points(self):
		points_seq = [0.2,0.5,0,2,0.5,0,1.5,-0.5,0]
		self.angles = [90, 0, 180, 45]
		quat_seq = list()
		for angle in self.angles:
			quat_seq.append(Quaternion(*(quaternion_from_euler(0, 0, angle*math.pi/180, axes='sxyz'))))

		n = 3
		points = [points_seq[i:i+n] for i in range(0, len(points_seq), n)]

		for point in points:
			self.pose_seq.append(Pose(Point(*point),quat_seq[n-3]))
			n += 1
	

  
if __name__ == '__main__':
	rostest.rosrun('botsandus_planner', 'planner_test', Testbostandus_planner, sys.argv)

