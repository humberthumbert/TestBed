#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from std_msgs.msg import String
import cv2
import socket

from cv_bridge import CvBridge
import numpy as np

class TimeStreamer:
        i = 0;
	def __init__(self):
		self.times = {}
		self.last_accurate_frame = 0
		self.subscriber2 = rospy.Subscriber("/pixel2/image_raw", Image, self.callback, "2", queue_size=1)
		self.publisher_pixel2 = rospy.Publisher("/pixel2/image_timed", Image, queue_size=1)

	def callback(self, ros_data, args):
		frame_number = ros_data.header.frame_id
		timestamp = 0 
		try:
			timestamp = self.times[str(frame_number)]
			self.last_accurate_frame = frame_number
		except:
			timestamp = self.times[self.last_accurate_frame]
                msg = ros_data

		cv_image = bridge.imgmsg_to_cv2(ros_data, "bgr8")#
		#img_arr = np.array(bytearray(msg),dtype=np.uint8)
		#img = cv2.imdecode(img_arr, -1)
                
		cv2.imshow('video',cv_image)
                cv2.waitKey(3)
		if (self.i%100==0):
                    print("Saved_image")
                    cv2.imwrite("/home/stamatis/my_ws/src/camera_node/src/"+str(self.i/100)+".png", cv_image)
                self.i=self.i+1

		msg.header.stamp = rospy.Time.from_sec((timestamp*1000)/1000000)
		self.publisher_pixel2.publish(msg)

	def getTimers(self):
		HOST = 'pixel2'  # The server's hostname or IP address
		PORT = 9192		# The port used by the server

		s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		s.connect((HOST, PORT))
		s.sendall(b'')
		prev_timestamp = 0
		while(True):
			data = s.recv(1024)
			frametimes = str(repr(data)).replace("b'", "").replace("'", "")
			l = filter(None, frametimes.split("<---->"))
			for i in l:
				frame_id = i.split(" ")[0]
				try:
					curr_timestamp = int(i.split(" ")[1])
					self.times[frame_id] = curr_timestamp
					prev_timestamp = curr_timestamp
				except:
					self.times[frame_id] = prev_timestamp


def main():
        global bridge#
	bridge = CvBridge()#

	rospy.init_node('timer_2', anonymous=True)
	ts = TimeStreamer()
	ts.getTimers()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down ROS timer node"

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
