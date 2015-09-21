import cv2
import cv
import numpy as np
from threading import Thread
from multiprocessing import Queue
import time
import imutils

class Tracker:
	def __init__(self):
		self.capture = cv2.VideoCapture(0)
		self.color_mid = np.array([0,0,0])
		self.current_image = Queue()
		self.c_tracker = Thread(target=self.continues_tracker, args=())
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

	def motionBased(self, image): # returns the boundingboxes of areas in motion, may only work in continus tracking
		return (-1,-1)

	def findCircles(self, image, minDist): 	#finds circles, warning: this takes a lot of cpu
		img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		gray = cv2.GaussianBlur(img, (5,5),5)
		circles = cv2.HoughCircles(gray, cv2.cv.CV_HOUGH_GRADIENT, 1.2, minDist)
		if circles is not None:
			circles = np.round(circles[0, :]).astype("int")
			return circles  				#this is a list
		else:
			return None

	def continues_tracker(self):
		points_colorbased = list()
		counter = 3
		circles = None
		working_copy = None
		firstFrame = None
		while 1:
			okay, image = self.capture.read()
			# 1. step: color_tracking
			ctr_colorbased = self.colorBased(image,np.array([40,70,0]),np.array([80,200,200]))
			if ctr_colorbased is not (-1,-1):
				points_colorbased.append(ctr_colorbased)
			else:
				points_colorbased.append(self.current_point)
			# 2. step: find circles, quite expensive
			if counter % 2 == 0:
				circles = self.findCircles(image,100)

			# 3. step: motion tracking:
			working_copy = image
			gray = cv2.cvtColor(working_copy, cv2.COLOR_BGR2GRAY)
			gray = cv2.GaussianBlur(gray, (5, 5), 0)
			if firstFrame is None:
				firstFrame = gray
			frameDelta = cv2.absdiff(firstFrame, gray)
			thresh = cv2.threshold(frameDelta, 25, 255, cv2.THRESH_BINARY)[1]
			thresh = cv2.erode(thresh, None, 10)
			thresh = cv2.dilate(thresh, None, 18)
			(contour, _) = cv2.findContours(thresh.copy(), cv.CV_RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
			
			if counter == 3:
				self.current_point = self.mean(points_colorbased)
				if circles is not None:
					for (x, y, r) in circles:
						if self.in_circle(x,y,r,self.current_point[0],self.current_point[1]):
							self.current_point = (x,y)
				del points_colorbased[:]
				counter = 0
				firstFrame = None

			if counter%2 == 0:
				if ctr_colorbased[0] != None and ctr_colorbased[1] != None:
					cv2.circle(image, ctr_colorbased, 10, (0,0,255))
				if circles is not None:
					for (x, y, r) in circles:
						cv2.circle(image, (x, y), r, (0, 255, 0), 4)
				cnts = list()
				for c in contour:
					area = cv2.contourArea(c)
					(x, y, w, h) = cv2.boundingRect(c)
					cv2.circle(image, (x+h/2, y+w/2), 10, (255, 0, 0), 2)
				cv2.circle(image, self.current_point, 20, (255,0,255),4)
				if self.current_image.empty():
					self.current_image.put(image)
			counter +=1

	def in_circle(self,center_x, center_y, radius, x, y):
	    square_dist = (center_x - x) ** 2 + (center_y - y) ** 2
	    return square_dist <= radius ** 2

	def mean(self,points):
		x,y = 0,0
		for a in points:
			x += a[0]
			y += a[1]
		return (x/len(points),y/len(points))

	def start_cont_tracker(self):
		if not self.c_tracker.isAlive():
			self.c_tracker.start()
		else:
			print "c_tracker already runs"
		time.sleep(1)

	def stop_cont_tracker(self):
		if self.c_tracker.isAlive():
			self.c_tracker.stop()
		else:
			print "c_tracker does not run"

	def get_current_point(self):
		return self.current_point

	def get_current_image(self):
		return self.current_image

	def track(self,image):
	    ctr_colorbased = self.colorBased(image,np.array([40,70,70]),np.array([80,200,200])) # calibrated for bright green
	    circles = self.findCircles(image,100)
	    if ctr_colorbased[0] != None and ctr_colorbased[1] != None:  						# mark colorbased trackpoint
	        cv2.circle(image, ctr_colorbased, 10, (0,0,255))
	    if circles is not None:
		    for (x, y, r) in circles:
				cv2.circle(image, (x, y), r, (0, 255, 0), 4)
	    return ctr_colorbased,image

	def getAbsolutePosition(self):
		okay, image = self.capture.read()
		return self.track(image)