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

class ColorDetector:
    def __init__(self):
        pass
    def detect(self, image):
        #transfer RGB to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        #define range of the color in HSV
        #red
        lower_red = np.array([156, 80, 99])
        upper_red = np.array([180, 255, 255])
        #green
        lower_green = np.array([35, 110, 200])
        upper_green = np.array([99, 255, 255])
        #blue
        lower_blue = np.array([90, 100, 100])
        upper_blue = np.array([124, 255, 255])
        #Threshold of HSV image to get the color
        mask_red   = cv2.inRange(hsv, lower_red, upper_red)
        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        mask_blue  = cv2.inRange(hsv, lower_blue, upper_blue)
        #final mask 
        mask1 = cv2.bitwise_or(mask_red, mask_green)
        mask  = cv2.bitwise_or(mask1, mask_blue)
        target = cv2.bitwise_and(image, image, mask=mask)
        target = cv2.dilate(target, None, iterations = 2)
        target = cv2.erode(target, None, iterations = 2)
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
            shape = "square" if abs(w-h) <= 10  else "rect"
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

	def prediction_cb(self, req):
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
		img     = img[:, :, ::-1]  # switch to BGR

		img = np.transpose(img, (2, 0, 1)) / 255.
		img[0] -= self.means[0]
		img[1] -= self.means[1]
		img[2] -= self.means[2]

		now = rospy.get_time()
		# convert to tensor
		img = img[np.newaxis,:]
		img = torch.from_numpy(img.copy()).float() 

		output = self.fcn_model(img)
		output = output.data.cpu().numpy()

		N, _, h, w = output.shape
		mask = output.transpose(0, 2, 3, 1).reshape(-1, self.n_class).argmax(axis = 1).reshape(N, h, w)[0]


		rospy.loginfo("Predict time : %f", rospy.get_time() - now)
		now = rospy.get_time()

		show_img = np.asarray(origin)
		count = np.zeros(3)
		self.mask1[:,:] = 0
		self.mask1[mask != 0] = 1
		labels = self.adj(self.mask1)
		mask = np.asarray(mask, np.uint8)

		cd      = ColorDetector()
		target  = cd.detect(self.mask1)
		gray    = cv2.cvtColor(target, cv2.COLOR_BGR2HSV)
		blurred = cv2.GaussianBlur(gray, (5,5), 0)
		img     = cv2.Canny(blurred, 20 160)
		cnts    = cv2.findContours(self.img.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		cnts    = cnts[1]
		sd      = ShapeDetector()
		for c in cnts:
		    shape = sd.detect(c)
		    if shape is "square":
		        M = cv2.moments(c)
		        if M["m00"] == 0:
		            break
		        cX = int(M["m10"] / M["m00"]) 
		        cY = int(M["m01"] / M["m00"]) 
		        # multiply the contour (x, y)-coordinates by the resize ratio,
		        # then draw the contours and the name of the shape on the image
		        cv2.drawContours(img, [c], 0, (255, 0, 0), 2)
		        cv2.putText(img, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (255, 255, 0), 2)
		        square += 1
		        p = labels[c[0,0,1],c[0,0,0]]
		        mask2[labels == p] = 5
		    if shape is "rect":
		        M = cv2.moments(c)
		        if M["m00"] == 0:
		            break
		        cX = int(M["m10"] / M["m00"]) 
		        cY = int(M["m01"] / M["m00"]) 
		        # multiply the contour (x, y)-coordinates by the resize ratio,
		        # then draw the contours and the name of the shape on the image
		        cv2.drawContours(img, [c], 0, (255, 0, 0), 2)
		        cv2.putText(img, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (255, 255, 0), 2)
		        rectangle += 1
		        p = labels[c[0,0,1],c[0,0,0]]
		        mask2[labels == p] = 4
		    if shape is "circle":
		        M = cv2.moments(c)
		        if M["m00"] == 0:
		            break
		        cX = int(M["m10"] / M["m00"]) 
		        cY = int(M["m01"] / M["m00"]) 
		        # multiply the contour (x, y)-coordinates by the resize ratio,
		        # then draw the contours and the name of the shape on the image
		        cv2.drawContours(img, [c], 0, (255, 0, 0), 2)
		        cv2.putText(img, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (255, 255, 0), 2)
		        circle += 1
		        p = labels[c[0,0,1],c[0,0,0]]
		        mask2[labels == p] = 6
		rospy.loginfo("Image processing time : %f", rospy.get_time() - now)
		cv2.imwrite("/home/andyser/Desktop/res.jpg", show_img)
		resp.process_image = self.cv_bridge.cv2_to_imgmsg(show_img, "bgr8")
		resp.mask = self.cv_bridge.cv2_to_imgmsg(mask2, "64FC1")
		
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

