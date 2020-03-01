#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge

import numpy as np
from urllib2 import urlopen
URL = "http://146.169.179.6:8080/shot.jpg"

import time

class image_streamer:

	def __init__(self):
		self.publisher_pixel1 = rospy.Publisher("/pixel1/image_rect_color", Image, queue_size=1)
		self.publisher_pixel2 = rospy.Publisher("/pixel2/image_rect_color", Image, queue_size=1)
		self.publisher_pixel3 = rospy.Publisher("/pixel3/image_rect_color", Image, queue_size=1)
		self.publisher_pixel4 = rospy.Publisher("/pixel4/image_rect_color", Image, queue_size=1)
		self.publisher_info = rospy.Publisher("/cam3d/rgb/camera_info", CameraInfo, queue_size=1)
	        
		self.subscriber1 = rospy.Subscriber("pixel1/image_raw", Image, self.callback, "1", queue_size=1)
		self.subscriber2 = rospy.Subscriber("pixel2/image_raw", Image, self.callback, "2", queue_size=1)
		self.subscriber3 = rospy.Subscriber("pixel3/image_raw", Image, self.callback, "3", queue_size=1)
		self.subscriber4 = rospy.Subscriber("pixel4/image_raw", Image, self.callback, "4", queue_size=1)


	def callback(self, ros_data, args):

		global bridge

		#img = bridge.compressed_imgmsg_to_cv2(ros_data, desired_encoding="bgr8")
		msg = ros_data
		#msg = bridge.cv2_to_imgmsg(img)
		msg.header.stamp = ros_data.header.stamp
		if(args[0]=="1"):
			ims.publisher_pixel1.publish(msg)
		if(args[0]=="2"):
			ims.publisher_pixel2.publish(msg)
		if(args[0]=="3"):
			ims.publisher_pixel3.publish(msg)
		if(args[0]=="4"):
			ims.publisher_pixel4.publish(msg)

		camera_info = CameraInfo()
		#camera_info.P = [515.4,   0.0, 323.0, 0.0,
                #		0.0, 518.7, 233.9, 0.0,
                #		0.0,   0.0,   1.0, 0.0]

		#camera_info.P = [3.947343969791328e+02, 0.0, 2.458620661682394e+02, 0.0,
		#		0.0, 3.973312505665456e+02, 1.587019280699527e+02, 0.0,
		#		0.0 ,0.0 , 1, 0.0]

		camera_info.P = [253.5595, 0.0, 157.5681, 0.0,
				0.0, 254.8321, 118.2755, 0.0,
				0.0 ,0.0 , 1, 0.0]

		camera_info.header.stamp = ros_data.header.stamp
		#camera_info.header.stamp = time.time()
		ims.publisher_info.publish(camera_info)

		#Callback function of subscribed topic. 
		#Here images get converted and features detected'''
		#if VERBOSE :

		##img_arr = np.array(bytearray(urlopen(URL).read()),dtype=np.uint8) #1
		##img = cv2.imdecode(img_arr,-1) #1
		#img = cv2.imread('myimg.png') #2
		##cv2.imshow('test',img)
		#cv2.waitKey()
		#frame = bridge.compressed_imgmsg_to_cv2(ros_data, desired_encoding="bgr8")
		##msg = bridge.cv2_to_imgmsg(img) #1
		#msg.header.stamp = rospy.Time.now() #ros_data.header.stamp
		#msg.format = "png"
		
		# Publish the image
		##self.publisher.publish(msg)

		#while True:
		#	self.publisher.publish(msg)
		#	cv2.waitKey()

def stream_publish():
	ims = image_streamer()
	while True:
		img_arr = np.array(bytearray(urlopen(URL).read()),dtype=np.uint8)
		img = cv2.imdecode(img_arr, -1)
		#cv2.imshow('test',img)
		#if cv2.waitKey(1)&0xFF == ord('q'):
		#	break
		
		msg = bridge.cv2_to_imgmsg(img)
		msg.header.stamp = time.time()
		ims.publisher_cam.publish(msg)

		camera_info = CameraInfo()
		camera_info.P = [515.4,   0.0, 323.0, 0.0,
                		0.0, 518.7, 233.9, 0.0,
                		0.0,   0.0,   1.0, 0.0]
		#camera_info.header.stamp = data.header.stamp
		camera_info.header.stamp = time.time()
		ims.publisher_info.publish(camera_info)
def main():

	global bridge
	bridge = CvBridge()

	#start node
	rospy.init_node('publisher', anonymous=True)

	global ims
	ims = image_streamer()
	
	#keep the node running until a KeyboardInterrupt is sent.
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down ROS camera1 node"
		cv2.destroyAllWindows()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
