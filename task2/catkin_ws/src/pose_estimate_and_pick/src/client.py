#!/usr/bin/env python

from pose_estimate_and_pick.srv import *
from object_detection.srv import *
import rospy
import geometry_msgs.msg
from math import pi,sqrt
from std_msgs.msg import String,Float64, Bool

class client_task2(object):
	def __init__(self):


		self.task2 = rospy.Service('task2', task2_srv, self.task2_ser)
		
	def task2_ser(self, req):

		pose_msg = rospy.ServiceProxy('pose_estimation', pose_estimation)
		

	def onShutdown(self):
		rospy.loginfo("Shutdown.")

if __name__ == '__main__': 
	rospy.init_node('client_task2',anonymous=False)
	rospy.sleep(2)
	client_task2 = client_task2()
	rospy.on_shutdown(client_task2.onShutdown)
	rospy.spin()