import cv2
import numpy as np
import Tkinter as tk
from PIL import Image, ImageTk
from thread import start_new_thread
import time
from multiprocessing import Process, Queue
from Queue import Empty
import time, sys
sys.path.append("/Users/blank/crazyflie/crazyflie-clients-python/examples../lib") # Change this to your path
import cflib
from cflib.crazyflie import Crazyflie
import logging
from Tracker import Tracker


def worker_thread(tracker, label,data_queue):
	divide = 0
	mean = Queue()
	while(1):
		ctr,image = tracker.getAbsolutePosition()
		mean.put(ctr)
		if divide is 10:
			queue.put(image)
			x,y = 0,0
			for i in range(0,divide):
				a = mean.get()
				x+=a[0]
				y+=a[1]
			x /=divide
			y/=divide
			data_queue.put((x,y))
			divide = 0
		divide += 1
		time.sleep (15.0 / 1000.0) # aprox 30fps <- nope

def update_image(image_label, queue):
   frame = queue.get()
   im = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
   a = Image.fromarray(im)
   b = ImageTk.PhotoImage(image=a)
   image_label.configure(image=b)
   image_label._image_cache = b  # avoid garbage collection
   root.update()

def update_all(root, image_label, queue):
	if not queue.empty():
   		update_image(image_label, queue)
   	root.after(0, func=lambda: update_all(root, image_label, queue))

class DroneHandler():
	def __init__(self, queue):
		self.data_queue = queue
		cflib.crtp.init_drivers(enable_debug_driver=False)
		self._cf = Crazyflie()
		logging.basicConfig(level=logging.ERROR)

	def findme(self):
		print "Scanning interfaces for Crazyflies..."
		self.available = cflib.crtp.scan_interfaces()
		print self.available
		self.target_x, self.target_y = 550, 280

	def connectFirst(self):
		self._cf.connected.add_callback(self._connected)
		self._cf.disconnected.add_callback(self._disconnected)
		self._cf.connection_failed.add_callback(self._connection_failed)
		self._cf.connection_lost.add_callback(self._connection_lost)
		self._cf.open_link(self.available[0][0])
		print "Connecting to %s" % self.available[0][0]

	def _connected(self, link_uri):
		print "Unlocking copter"
		self._cf.commander.send_setpoint(0, 0, 0, 0)
		print "Starting controller"
		start_new_thread(self.controllCopter,(self.data_queue,))
		print "connected"

	def _connection_failed(self, link_uri, msg):
		print "Connection to %s failed: %s" % (link_uri, msg)

	def _connection_lost(self, link_uri, msg):
		print "Connection to %s lost: %s" % (link_uri, msg)

	def _disconnected(self, link_uri):
		print "Disconnected from %s" % self.link_uri
	def controllCopter(self, data):
		#self._cf.commander.send_setpoint(roll, pitch, yawrate, thrust)
		roll = 0
		thrust = 11000
		error_y = 0
		error_y_old = 0
		error_x = 0
		error_x_old = 0
		int_x = 0
		int_y = 0
		dev_x = 0
		dev_y = 0
		out_x = 0
		out_y = 0
		Kp = 4
		Ki = 1
		Kd = 4
		dt = 1
		while(1):
			a = data.get()
			if not a == (-1,-1):
				error_x = self.target_x - a[0]
				error_y = self.target_y - a[1]
				int_x = int_x + error_x*dt
				int_y = int_y + error_y*dt
				dev_x = (error_x - error_x_old) / dt
				dev_y = (error_y - error_y_old) / dt
				out_x = Kp*error_x + Ki*int_x + Kd*dev_x
				out_y = Kp*error_y + 1.3*int_y + Kd*dev_y
				error_x_old = error_x
				error_y_old = error_y

				thrust = 6*-out_y
				roll = -0.01*out_x
 				if thrust < 10000:
					thrust = 10000
				elif thrust > 50000:
					thrust = 50000
				else:
					pass
				if roll < -20:
					out = -20
				elif roll > 20:
					roll = 20
				else:
					pass
			self._cf.commander.send_setpoint(roll, 0, 0, thrust)
 			print str(thrust) + "		|		" + str(roll)
#550, 280

if __name__ == '__main__':
	queue = Queue()
	data_queue = Queue()
	tracker = Tracker()
	root = tk.Tk()
	root.geometry("1200x700")
	root.bind('<Escape>', lambda e: root.quit())
	label = tk.Label(root)
	label.pack(fill="both", expand="yes")
	start_new_thread(worker_thread,(tracker,label,data_queue,))
	# drone = DroneHandler(data_queue)
	# drone.findme()
	# # start_new_thread(drone.connectFirst(),())
	# drone.connectFirst()
	root.after(0, func=lambda: update_all(root, label, queue))
	root.mainloop()