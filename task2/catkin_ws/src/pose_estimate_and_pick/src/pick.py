#!/usr/bin/env python

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi,sqrt
from std_msgs.msg import String,Float64, Bool
from moveit_commander.conversions import pose_to_list
import rospy
import tf
from place_to_box.srv import *
import ik_4dof




class pick_node(object):
	def __init__(self):
		self.listener = tf.TransformListener()
		self.node_name = "place_node"
		check = True
		while(check):
			check = False
			try:
				self.move_group_arm = moveit_commander.MoveGroupCommander("arm")
			except:
				print "moveit server isn't open yet"
				check = True

		self.pub_gripper = rospy.Publisher("/gripper_joint/command",Float64,queue_size=1)

		self.place_srv = rospy.Service("pick", tag, self.transform)
		self.place_srv = rospy.Service("go_home",home, self.home)
		
	def home(self,req):
		joint_goal = self.move_group_arm.get_current_joint_values()
		joint_goal[0] = 0
		joint_goal[1] = -pi*5/13
		joint_goal[2] = pi*3/4
		joint_goal[3] = pi/3
		joint_goal[4] = 0
		self.move_group_arm.go(joint_goal, wait=True)
		return homeResponse("Home now!")		

	def onShutdown(self):
		rospy.loginfo("Shutdown.")


if __name__ == '__main__': 
	rospy.init_node('pick_node',anonymous=False)
	rospy.sleep(2)
	pick_node = pick_node()
	rospy.on_shutdown(pick_node.onShutdown)
	rospy.spin()