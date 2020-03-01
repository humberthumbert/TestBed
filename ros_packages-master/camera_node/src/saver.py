#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge

import numpy as np
from urllib2 import urlopen
URL = "http://146.169.149.166:8080/shot.jpg"

import time

def stream_publish():
	i = 0
	while True:
		i = i+1
		img_arr = np.array(bytearray(urlopen(URL).read()),dtype=np.uint8)
		img = cv2.imdecode(img_arr, -1)
		cv2.imwrite(str(i)+".png", img)
		cv2.imshow('video',img)
		if cv2.waitKey(0)&0xFF == ord('q'):
			break

def main():

	global bridge
	bridge = CvBridge()

	#start node
	rospy.init_node('saver', anonymous=True)
	
	stream_publish()

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
