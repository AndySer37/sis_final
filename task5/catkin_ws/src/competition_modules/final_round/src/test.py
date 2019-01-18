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
from geometry_msgs.msg import PoseStamped, Twist
from robot_navigation.srv import robot_navigation
from apriltags2_ros.msg import AprilTagDetection

# Competition ROS Srv
from robot_navigation.srv import robot_navigation
from object_detection.srv import task1out
from pose_estimate_and_pick.srv import task2_srv
from place_to_box.srv import home, tag

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
        self.odom_sub   = rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)
        self.tag_sub    = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.tag_cb, queue_size = 1)

        self.x = 0
        self.y = 0
        self.yaw = 0
        self.tags_insight = []
        self.target_tag = 0
        self.num_obj = 1 ###################
        self.fsm_state = 0

        self.timer = rospy.Timer(rospy.Duration(1), self.process)


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

    def fsm_transit(self, state_to_transit):
        self.fsm_state = state_to_transit

    def process(self, event):
        if self.fsm_state == 0:
            print 'Robot Initialization'
            try:    # Wait for rosservice ready
                rospy.wait_for_service(NAVIGATION_SRV,  timeout=5)
                rospy.wait_for_service(TASK1_SRV,       timeout=5)
                rospy.wait_for_service(TASK2_SRV,       timeout=5)
                rospy.wait_for_service(GRIP_PLACE_SRV,  timeout=5)
                rospy.wait_for_service(GRIP_CLOSE_SRV,  timeout=5)
                rospy.wait_for_service(GRIP_OPEN_SRV,   timeout=5)
                self.fsm_transit(1)

            except (rospy.ServiceException, rospy.ROSException), e:
                rospy.logerr('State:%2d, error: %s' % (self.fsm_state, e))
                self.fsm_transit(99)


        if self.fsm_state == 1:
            # Check whether tag 0 is in sight?
            self.target_tag = 0
            if len(self.tags_insight) != 0:
                for tags in self.tags_insight:
                    if tags.id[0] == self.target_tag:
                        self.fsm_transit(2)
                        break

            ''' Todo: if tag0 is not in sight '''
            rospy.logerr('Tag0 is not in sight!')
            self.fsm_transit(99)


        if self.fsm_state == 2:
            # Count the object number
            try:
                object_detect = rospy.ServiceProxy(TASK1_SRV, task1out)
                task1_resp = object_detect()
                
                
                ''' Todo: count the object number on the platform '''
                # img = self.cv_bridge.imgmsg_to_cv2(task1_resp.mask, "bgr8")

                # Finish task if no object on the platform
                if self.num_obj == 0:
                    self.fsm_transit(88)

                self.fsm_transit(3)
            except (rospy.ServiceException, rospy.ROSException), e:
                rospy.logerr('State:%2d, error: %s' % (self.fsm_state, e))
                self.fsm_transit(99)


        if self.fsm_state == 3:
            print 'predict and picking the object'
            try:
                predict_and_pick = rospy.ServiceProxy(TASK2_SRV, task2_srv)
                task2_resp = predict_and_pick()
                self.target_tag = int(task2_resp.tag_id)
                self.fsm_transit(4)
            except (rospy.ServiceException, rospy.ROSException), e:
                rospy.logerr('State:%2d, error: %s' % (self.fsm_state, e))
                self.fsm_transit(99)


        if self.fsm_state == 4:
            print 'Moving to target tag'
            try:
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
                place = rospy.ServiceProxy(GRIP_PLACE_SRV, tag)
                task4_resp = place(self.target_tag)
                ret_str = task4_resp.result
                if str1.find('Successful') == -1:
                    rospy.logerr(str1)
                    self.fsm_transit(99)
                    return
                rospy.loginfo(str1)
                grip_open = rospy.ServiceProxy(GRIP_OPEN_SRV, home)
                task4_resp = grip_open()
                rospy.sleep(1)
                grip_home = rospy.ServiceProxy(GRIP_HOME_SRV, home)
                task4_resp = grip_home()
                self.fsm_transit(6)

            except (rospy.ServiceException, rospy.ROSException), e:
                rospy.logerr('State:%2d, error: %s' % (self.fsm_state, e))
                self.fsm_transit(99)


        if self.fsm_state == 6:
            print 'Way home'
            try:
                self.target_tag = 0
                car_move = rospy.ServiceProxy(NAVIGATION_SRV, robot_navigation)
                task3_resp = car_move(self.target_tag)
                self.fsm_transit(0)
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

        rospy.sleep(0.5)

    def onShutdown(self):
        rospy.loginfo("Node shutdown")

def main(args):
    rospy.init_node('final_round_node', anonymous = True)
    task5 = final_round_node()
    rospy.on_shutdown(task5.onShutdown)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)