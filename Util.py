import cv2
import numpy as np
class Tracker:
	def __init__(self):
		self.capture = cv2.VideoCapture(0)
		self.color_mid = np.array([0,0,0])
		for i in range(1,10): 
			okay, image = self.capture.read() #init camera

	def colorBased(self,image,lower_color,upper_color):
		blur = cv2.GaussianBlur(image, (5,5),0)
		hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
		mask = cv2.inRange(hsv, lower_color, upper_color)
		bmask = cv2.GaussianBlur(mask, (5,5),0)
		moments = cv2.moments(bmask)
		m00 = moments['m00']
		centroid_x, centroid_y = None, None
		if m00 != 0:
		    centroid_x = int(moments['m10']/m00)
		    centroid_y = int(moments['m01']/m00)
		ctr = (-1,-1)
		if centroid_x != None and centroid_y != None:
		    ctr = (centroid_x, centroid_y)
		return ctr

	def motionBased(self, image): # returns the boundingboxes of areas in motion
		return -1

	def findCircles(self, image, minDist): #finds circles, warning: this takes a lot of cpu
		img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		gray = cv2.GaussianBlur(img, (5,5),5)
		circles = cv2.HoughCircles(gray, cv2.cv.CV_HOUGH_GRADIENT, 1.2, mindDist)
		if circles is not None:
			circles = np.round(circles[0, :]).astype("int")
			return circles  #this is a list
		else:
			return None


	def track(self,image):
	    ctr_colorbased = self.colorBased(image,np.array([40,70,70]),np.array([80,200,200])) # calibrated for bright green
	    if ctr_colorbased[0] != None and ctr_colorbased[1] != None:  # mark colorbased trackpoint
	        cv2.circle(image, ctr_colorbased, 10, (0,0,255))
	    return ctr_colorbased

	def getAbsolutePosition(self):
		okay, image = self.capture.read()
		return self.track(image)