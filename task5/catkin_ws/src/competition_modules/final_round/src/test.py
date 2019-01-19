#!/usr/bin/env python
import sys
import time
from math import pi
import numpy as np

import rospy
import actionlib
import tf
import cv2
from cv_bridge import CvBridge, CvBridgeError

# ROS Msg
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Int32, Float64, Bool
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from robot_navigation.srv import robot_navigation
from apriltags2_ros.msg import AprilTagDetectionArray, AprilTagDetection

# Competition ROS Srv
from robot_navigation.srv import robot_navigation
from object_detection.srv import task1out
from pose_estimate_and_pick.srv import task2_srv
from place_to_box.srv import home, tag
from std_srvs.srv import *

# Available service name
NAVIGATION_SRV  = 'robot_navigate'
TASK1_SRV       = 'prediction'
TASK2_SRV       = 'task2'
GRIP_PLACE_SRV  = 'place_to_box'
GRIP_HOME_SRV   = 'home'
GRIP_CLOSE_SRV  = 'close'
GRIP_OPEN_SRV   = 'open_grip'


class final_round_node():
    def __init__(self):
        self.cv_bridge = CvBridge()
        # self.odom_sub   = rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)
        self.tag_sub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.tag_cb, queue_size = 1)
        self.cmd_pub = rospy.Publisher('cmd_vel', TwistStamped, queue_size=1)
        self.place_srv = rospy.Service("final_task_trigger", Trigger, self.final_trigger_cb)

        self.x = 0
        self.y = 0
        self.yaw = 0
        self.tags_insight = []
        self.target_tag = 5 # face to object platform
        self.num_obj = 3 
        self.fsm_state = 0

        self.motion_trigger = False

        # self.timer = rospy.Timer(rospy.Duration(1), self.process)

    def final_trigger_cb(self, req):
        self.motion_trigger = not self.motion_trigger
        if self.motion_trigger == True:
            rospy.loginfo('Final Task state:%2d Run' % self.fsm_state)
        else: rospy.loginfo('Final Task state:%2d Stop' % self.fsm_state)
        resp = TriggerResponse()
        resp.success = True
        return resp

    def tag_cb(self, msg):
        self.tags_insight = msg.detections

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = [ msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, \
              msg.pose.pose.orientation.z, msg.pose.pose.orientation.w ]
        euler = tf.transformations.euler_from_quaternion(q)
        self.yaw = euler[2]
        # print self.x, self.y, self.yaw

    def tag_detection(self, target_id):
        if len(self.tags_insight) == 0:
            return False
        else:    
            for tags in self.tags_insight:
                if tags.id[0] == self.target_id: return True           

    def fsm_transit(self, state_to_transit):
        self.fsm_state = state_to_transit

    def process(self):
        if self.fsm_state == 0:
            print 'Robot Initialization'
            try:    # Wait for rosservice ready
                rospy.wait_for_service(NAVIGATION_SRV)
                print 'go to state 2' #######################3
                self.fsm_transit(2)

            except (rospy.ServiceException, rospy.ROSException), e:
                rospy.logerr('State:%2d, error: %s' % (self.fsm_state, e))
                # self.fsm_transit(99)


        if self.fsm_state == 1:
            # Check whether tag 5 is in sight?
            self.target_tag = 5
            stat = rospy.wait_for_message("/odom", Odometry)
            theta = stat.twist.twist.angular.z  
            
            if self.tag_detection(self.target_tag) == False:
                cmd = Twist()
                cmd.angular.z = theta/abs(theta) * 0.1
                self.pub_cmd.publish(cmd)
                rospy.logerr('Tag%2d is not in sight!' % self, target_id)
            else:
                print 'got tag 5'
                cmd = Twist()
                cmd.angular.z = 0
                self.pub_cmd.publish(cmd)
                self.fsm_transit(2)


        if self.fsm_state == 2:
            # Count the object number to check whether the robot finished the task
            try:
                if self.num_obj == 0:
                    self.fsm_transit(88)
                    return
                self.fsm_transit(3)
            except (rospy.ServiceException, rospy.ROSException), e:
                rospy.logerr('State:%2d, error: %s' % (self.fsm_state, e))
                self.fsm_transit(99)


        if self.fsm_state == 3:
            print 'predicting and picking the object'
            try:
                rospy.wait_for_service(TASK2_SRV)
                predict_and_pick = rospy.ServiceProxy(TASK2_SRV, task2_srv)
                task2_resp = predict_and_pick()
                str1 = task2_resp.tag_id
                if str1.find('Fail') == -1: # if task2 success, get the id related the picked obj
                    self.target_tag = int(task2_resp.tag_id)
                    self.fsm_transit(5)######4)
                    print 'skipppppppppppp state 4'
                else: rospy.sleep(3)
            except (rospy.ServiceException, rospy.ROSException), e:
                rospy.logerr('State:%2d, error: %s' % (self.fsm_state, e))
                self.fsm_transit(99)


        if self.fsm_state == 4:
            print 'Moving to target tag'
            try:
                rospy.wait_for_service(NAVIGATION_SRV)
                car_move = rospy.ServiceProxy(NAVIGATION_SRV, robot_navigation)
                task3_resp = car_move(self.target_tag)
                self.fsm_transit(5)
                rospy.sleep(2)
            except (rospy.ServiceException, rospy.ROSException), e:
                rospy.logerr('State:%2d, error: %s' % (self.fsm_state, e))
                self.fsm_transit(99)


        if self.fsm_state == 5:
            print 'Placing the object'
            try:
                rospy.wait_for_service(GRIP_PLACE_SRV)
                place = rospy.ServiceProxy(GRIP_PLACE_SRV, tag)
                task4_resp = place(self.target_tag)
                ret_str = task4_resp.result
                if str1.find('Successful') == -1:
                    rospy.logerr(str1)
                    '''
                    Todo: deal with condition of pick fail
                    '''
                    self.fsm_transit(99)
                    return
                rospy.loginfo(str1)
                self.num_obj = self.num_obj - 1
                self.fsm_transit(1)
                print 'skipppppppppppp state 6'

            except (rospy.ServiceException, rospy.ROSException), e:
                rospy.logerr('State:%2d, error: %s' % (self.fsm_state, e))
                self.fsm_transit(99)


        if self.fsm_state == 6:
            print 'Way home'
            try:
                self.target_tag = 5
                car_move = rospy.ServiceProxy(NAVIGATION_SRV, robot_navigation)
                task3_resp = car_move(self.target_tag)
                self.fsm_transit(1)
                rospy.sleep(2)
            except (rospy.ServiceException, rospy.ROSException), e:
                rospy.logerr('State:%2d, error: %s' % (self.fsm_state, e))
                self.fsm_transit(99)


        if self.fsm_state == 88:
            rospy.loginfo('Finish the tasks!')
            self.timer.shutdown()
                

        if self.fsm_state == 99:    # Error state
            print('Node error')
            rospy.sleep(5)

        rospy.sleep(1.0)

    def onShutdown(self):
        rospy.loginfo("Node shutdown")

def main(args):
    rospy.init_node('final_round_node', anonymous = True)
    task5 = final_round_node()
    rospy.on_shutdown(task5.onShutdown)

    while not rospy.is_shutdown():
        if task5.motion_trigger == True:
            task5.process()
        rospy.sleep(1.0)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)