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
		time.sleep(0.01)
		data_queue.put(tracker.get_current_point())

def update_image(image_label, queue):
   im = queue.get()
   a = Image.fromarray(im)
   a = a.resize((650, 365), Image.NEAREST)
   b = ImageTk.PhotoImage(image=a)
   image_label.configure(image=b)
   image_label._image_cache = b  # avoid garbage collection
   root.update()

def update_all(root, image_label, queue):
	if not image_queue.empty():
		update_image(image_label, image_queue)
   	root.after(0, func=lambda: update_all(root, image_label, queue))

def key(event):
	#TODO redo me!
	if event.char is 'w':
		drone.pitch = 10
	elif event.char is 'a':
		drone.roll = 10
	if event.char is 'q':
		drone.yaw = -10
	elif event.char is 'e':
		drone.yw = 10
	elif event.char is 's':
		drone.pitch = -10
	elif event.char is 'd':
		drone.roll = -10
	elif event.char is ' ':
		drone.thrust = 40000
	elif event.char is 'p':
		drone.is_engaged = not drone.is_engaged
		exit()
	else:
		pass

def key_release(event):
	#TODO redo me!
	if event.char is 'w':
		drone.pitch = 0
	elif event.char is 'a':
		drone.roll = 0
	if event.char is 'q':
		drone.yaw = 0
	elif event.char is 'e':
		drone.yw = 0
	elif event.char is 's':
		drone.pitch = 0
	elif event.char is 'd':
		drone.roll = 0
	elif event.char is ' ':
		drone.thrust = 0
	else:
		pass

if __name__ == '__main__':
	data_queue = Queue()

	tracker = Tracker(mode = "2D")
	tracker.start_cont_tracker()
	image_queue=tracker.get_current_image()
	#image_queue2=tracker.get_current_image2()
	drone = DroneHandler(data_queue)

	root = tk.Tk()
	root.geometry("1300x400")
	root.bind('<Escape>', lambda e: exit())
	root.bind("<Key>", key)
	root.bind("<KeyRelease>", key_release)
	label = tk.Label(root)
	#label2 = tk.Label(root)
	label.pack(side = tk.LEFT)
	#label2.pack(side = tk.RIGHT)

	#start_new_thread(worker_thread,(tracker,label,data_queue,))

	#drone.findme()
	#drone.connectFirst()

	root.after(0, func=lambda: update_all(root, label, image_queue))
	root.mainloop()
