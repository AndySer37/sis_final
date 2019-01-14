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


class place_node(object):
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

		self.place_srv = rospy.Service("place_to_box", tag, self.transform)
		self.place_srv = rospy.Service("home_place",home, self.home)

	def transform(self,req):
		br = tf.TransformBroadcaster()
		tag = "tag_" + str(req.tag_id)
		try:
			now = rospy.Time.now()
			self.listener.waitForTransform('pi_camera', tag, now, rospy.Duration(3.0))
			(trans, rot) = self.listener.lookupTransform('pi_camera', tag , now)
			

		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, \
			tf.Exception):
			rospy.loginfo("Tag not found!")
			return tagResponse("Tag not found!")

		pose_goal = geometry_msgs.msg.Pose()
		
		pose_goal.position.x = trans[2] - 0.23
		pose_goal.position.y = - trans[0]
		pose_goal.position.z = trans[1] + 0.02


		print "Your box's position : " , pose_goal.position

		joint_value = ik_4dof.ik_solver(pose_goal.position.x, pose_goal.position.y, pose_goal.position.z, -90)

                for joint in joint_value:
                        joint = list(joint)
                        # determine gripper state
                        joint.append(0)
                        try:
                                self.move_group_arm.go(joint, wait=True)
                        except:
                                rospy.loginfo(str(joint) + " isn't a valid configuration.")

                grip_data = Float64()
                grip_data.data = 0.5
                self.pub_gripper.publish(grip_data)
                rospy.sleep(2)
                self.home(home)
                rospy.sleep(2)
                grip_data.data = 2.0
                self.pub_gripper.publish(grip_data)
                rospy.loginfo("End process")
                return tagResponse("Process Successfully")


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
		self.loginfo("Shutdown.")

if __name__ == '__main__': 
	rospy.init_node('place_node',anonymous=False)
	rospy.sleep(2)
        place_node = place_node()
	rospy.on_shutdown(place_node.onShutdown)
	rospy.spin()

