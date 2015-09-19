import numpy as np
import cv2
import time
capture = cv2.VideoCapture(0)
for i in range(1,10):
	okay, image = capture.read()
while(1):
	okay, image = capture.read()
	img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	gray = cv2.GaussianBlur(img, (5,5),5)
	circles = cv2.HoughCircles(gray, cv2.cv.CV_HOUGH_GRADIENT, 1.2, 150)
	if circles is not None:
		circles = np.round(circles[0, :]).astype("int")
		for (x, y, r) in circles:
			cv2.circle(gray, (x, y), r, (0, 255, 0), 4)
	cv2.imshow("asd",gray)
	cv2.waitKey(0)