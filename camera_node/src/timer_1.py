#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CompressedImage, Image, CameraInfo, PointCloud2
from std_msgs.msg import String
import cv2
import socket

class TimeStreamer:
	def __init__(self):
		self.times = {}
		self.last_accurate_frame = 0
		self.l_to_p = 0
		self.l_to_v = 0
		self.offset = 0
		self.velodyne_sub = rospy.Subscriber("velodyne_points", PointCloud2, self.callback_velodyne, "1", queue_size=1)
		self.subscriber = rospy.Subscriber("pixel1/image_raw", Image, self.callback, "1", queue_size=1)
		self.publisher_pixel = rospy.Publisher("/pixel1/image_timed", Image, queue_size=1)

	def callback_velodyne(self, ros_data, args):
		self.l_to_v = rospy.Time.now().to_nsec() - ros_data.header.stamp.to_nsec()

	def callback(self, ros_data, args):
		frame_number = ros_data.header.frame_id
		timestamp = 0
		try:
			timestamp = self.times[str(frame_number)]
			self.last_accurate_frame = frame_number
		except:
			timestamp = self.times[self.last_accurate_frame]
		precise_time = rospy.Time.from_sec(float(timestamp)/1000)
		self.l_to_p = ros_data.header.stamp.to_nsec() - precise_time.to_nsec()
		self.offset = self.l_to_p - self.l_to_v
		msg = ros_data
		msg.header.stamp = rospy.Time.from_sec(float(precise_time.to_nsec() + self.offset)/1000000000)
		self.publisher_pixel.publish(msg)

	def getTimers(self):
		HOST = 'pixel1'  # The server's hostname or IP address
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
	rospy.init_node('timer_1', anonymous=True)

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
