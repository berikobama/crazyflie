import cv2
import numpy as np
import Tkinter as tk
from PIL import Image, ImageTk
from thread import start_new_thread
import time
from multiprocessing import Process, Queue
from Queue import Empty
import time, sys

from Util import Tracker
from Util import DroneHandler

def worker_thread(tracker, label,data_queue):
	while 1:
		time.sleep(0.1)
		data_queue.put(tracker.get_current_point())

def update_image(image_label, queue):
   frame = queue.get()
   #im = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
   im =frame
   a = Image.fromarray(im)
   b = ImageTk.PhotoImage(image=a)
   image_label.configure(image=b)
   image_label._image_cache = b  # avoid garbage collection
   root.update()

def update_all(root, image_label, image_queue):
	if not image_queue.empty():
   		update_image(image_label, image_queue)
   	root.after(0, func=lambda: update_all(root, image_label, image_queue))

def key(event):
	# self.yaw = 0
	# self.thrust = 0
	if event.char is 'w':
		drone.pitch = 10
	elif event.char is 'a':
		drone.roll = 10
	elif event.char is 's':
		drone.pitch = -10
	elif event.char is 'd':
		drone.roll = -10
	elif event.char is 'p':
		drone.thrust = 0
		exit()
	else:
		print "asd"

def key_release(event):
	# self.yaw = 0
	# self.thrust = 0
	if event.char is 'w':
		drone.pitch = 0
	elif event.char is 'a':
		drone.roll = 0
	elif event.char is 's':
		drone.pitch = 0
	elif event.char is 'd':
		drone.roll = 0
	elif event.char is 'p':
		drone.thrust = 0
		exit()
	else:
		print "asd"

if __name__ == '__main__':
	data_queue = Queue()

	tracker = Tracker()
	tracker.start_cont_tracker()
	image_queue=tracker.get_current_image()

	drone = DroneHandler(data_queue)

	root = tk.Tk()
	root.geometry("1300x750")
	root.bind('<Escape>', lambda e: exit())
	root.bind("<Key>", key)
	root.bind("<KeyRelease>", key_release)
	label = tk.Label(root)
	label.pack(fill="both", expand="yes")

	start_new_thread(worker_thread,(tracker,label,data_queue,))

	#drone.findme()
	#drone.connectFirst()

	root.after(0, func=lambda: update_all(root, label, image_queue))
	root.mainloop()
