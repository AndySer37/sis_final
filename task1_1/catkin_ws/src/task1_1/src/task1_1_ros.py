#!/usr/bin/env python

import torch
import torch.nn as nn
import torch.optim as optim
import torchvision
from torchvision import models
from torchvision.models.vgg import VGG
import random
import sys

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String,Float64, Bool
from sensor_msgs.msg import Image, CameraInfo, PointCloud2

from torch.optim import lr_scheduler
from torch.autograd import Variable
from torch.utils.data import Dataset, DataLoader

import numpy as np
import time
import os

from task1_1.srv import *

class ColorDetector:
    def __init__(self):
        pass
    def detect(self, image):
        #transfer RGB to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        #define range of the color in HSV
        #red
        lower_red = np.array([156, 80, 90])
        upper_red = np.array([180, 255, 255])
        #green
        lower_green = np.array([35, 80, 140])
        upper_green = np.array([99, 255, 255])
        #blue
        lower_blue = np.array([90, 110, 120])
        upper_blue = np.array([124, 255, 255])
        #Threshold of HSV image to get the color
        mask_red   = cv2.inRange(hsv, lower_red, upper_red)
        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        mask_blue  = cv2.inRange(hsv, lower_blue, upper_blue)
        #final mask 
        mask1 = cv2.bitwise_or(mask_red, mask_green)
        mask  = cv2.bitwise_or(mask1, mask_blue)
        target = cv2.bitwise_and(image, image, mask=mask)
        target = cv2.dilate(target, None, iterations = 4)
        target = cv2.erode(target, None, iterations = 4)
        return target

class ShapeDetector:
    def __init__(self):
        pass
    def detect(self, c):
        # initialize the shape name and approximate the contour
        shape = "unidentified"
        peri = cv2.arcLength(c, True)
        c1 = cv2.convexHull(c)
        approx = cv2.approxPolyDP(c1, 0.04 * peri, True)
        # if the shape is a triangle, it will have 3 vertices
        if len(approx) == 3:
            shape = "triangle"
        # if the shape has 4 vertices, it is either a square or
        # a rectangle
        elif len(approx) == 4:
            # compute the bounding box of the contour and use the
            # bounding box to compute the aspect ratio
            (x, y, w, h) = cv2.boundingRect(approx)
            ar = w / float(h)
            shape = "square" if ar >= 0.8 and ar <= 1.2  else "rect"
        else:
            shape = "circle"
        return shape

class task1_1(object):
	def __init__(self):
		self.predict_ser = rospy.Service("prediction", task1out, self.prediction_cb)
		self.cv_bridge = CvBridge()
		
		self.square = 0
		self.rectangle = 0
		self.circle = 0
		self.MAXAREA = 20000
		self.MINAREA = 600		
	

	def prediction_cb(self, req):
		self.square = 0
		self.rectangle = 0
		self.circle = 0
		resp = task1outResponse()	
		im_msg = rospy.wait_for_message('/camera/rgb/image_rect_color', Image, timeout=None)			
		resp.pc = rospy.wait_for_message('/camera/depth_registered/points', PointCloud2, timeout=None)
		rospy.loginfo("Get image.")
		resp.org_image = im_msg
		try:
			img = self.cv_bridge.imgmsg_to_cv2(im_msg, "bgr8")
		except CvBridgeError as e:
			print(e)
		origin  = img


		cd        = ColorDetector()
		target    = cd.detect(origin)
		gray      = cv2.cvtColor(target, cv2.COLOR_BGR2HSV)
		blurred   = cv2.GaussianBlur(gray, (5,5), 0)
		_, threshold = cv2.threshold(blurred, 240, 255, cv2.THRESH_BINARY)
		#img_canny = cv2.Canny(blurred, 20, 160)
		cnts      = cv2.findContours(threshold.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		cnts      = cnts[1]
		sd        = ShapeDetector()
		img2      = target.copy()
		for c in cnts:
		    shape = sd.detect(c)
		    if self.MAXAREA >= cv2.contourArea(c) >= self.MINAREA:
		   	if shape is "square":
				M = cv2.moments(c)
			        if M["m00"] == 0:
			            break
			        cX = int(M["m10"] / M["m00"]) 
			        cY = int(M["m01"] / M["m00"]) 
			        # multiply the contour (x, y)-coordinates by the resize ratio,
			        # then draw the contours and the name of the shape on the image
			        cv2.drawContours(img2, [c], 0, (255, 0, 0), 2)
			        cv2.putText(img, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (255, 255, 0), 2)
			        self.square += 1
			        
			if shape is "rect":
				M = cv2.moments(c)
			        if M["m00"] == 0:
			            break
			        cX = int(M["m10"] / M["m00"]) 
			        cY = int(M["m01"] / M["m00"]) 
			        # multiply the contour (x, y)-coordinates by the resize ratio,
			        # then draw the contours and the name of the shape on the image
			        cv2.drawContours(img2, [c], 0, (255, 0, 0), 2)
			        cv2.putText(img2, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (255, 255, 0), 2)
			        self.rectangle += 1
		        
			if shape is "circle":
			        M = cv2.moments(c)
			        if M["m00"] == 0:
			            break
			        cX = int(M["m10"] / M["m00"]) 
			        cY = int(M["m01"] / M["m00"]) 
			        # multiply the contour (x, y)-coordinates by the resize ratio,
			        # then draw the contours and the name of the shape on the image
			        cv2.drawContours(img2, [c], 0, (255, 0, 0), 2)
			        cv2.putText(img, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (255, 255, 0), 2)
			        self.circle += 1
		        	
		resp.process_image = self.cv_bridge.cv2_to_imgmsg(img2, "bgr8")
		cv2.imwrite("/root/output.jpg", img2)
		print("circle", self.circle, "rectangle ", self.rectangle, "square ", self.square)
	def adj(self, _img, _level = 8):
		colomn, row = self.h, self.w
		_count = 0
		_pixel_pair = []
		label = np.zeros((colomn,row))
		for i in range(colomn):
			for j in range(row):
				if (_img[i,j] == 1 and label[i,j] == 0):
				    _pixel_pair.append([i,j])
				    _count += 1
				while len(_pixel_pair) != 0:
					pair = _pixel_pair.pop()
					a = pair[1] + 1
					b = pair[1] - 1
					c = pair[0] + 1
					d = pair[0] - 1
					if a == 640 : a -= 1
					if b == -1  : b += 1
					if c == 480 : c -= 1
					if d == -1  : d += 1

					if _img[pair[0],a] == 1 and label[pair[0],a] == 0:
					    _pixel_pair.append([pair[0],a])
					if _img[pair[0],b] == 1 and label[pair[0],b] == 0:
					    _pixel_pair.append([pair[0],b])
					if _img[c,pair[1]] == 1 and label[c,pair[1]] == 0:
					    _pixel_pair.append([c,pair[1]])
					if _img[d,pair[1]] == 1 and label[d,pair[1]] == 0:
					    _pixel_pair.append([d,pair[1]])
					if _level == 8:
						if _img[c,a] == 1 and label[c,a] == 0:
							_pixel_pair.append([c,a])
						if _img[d,a] == 1 and label[d,a] == 0:
							_pixel_pair.append([d,a])
						if _img[d,b] == 1 and label[d,b] == 0:
							_pixel_pair.append([d,b])
						if _img[c,b] == 1 and label[c,b] == 0:
							_pixel_pair.append([c,b])
					label[pair[0],pair[1]] = _count

			print("Num of classes for connected components : ", _count)
			return label

	def onShutdown(self):
		rospy.loginfo("Shutdown.")


if __name__ == '__main__': 
	rospy.init_node('task1_1',anonymous=False)
	task1_1 = task1_1()
	rospy.on_shutdown(task1_1.onShutdown)
	rospy.spin()

