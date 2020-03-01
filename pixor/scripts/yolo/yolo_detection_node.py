#!/usr/bin/env python
import rospy
import datetime
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from yolo_detector import *


class yolo_detection_node:
    def __init__(self):
        self.detector = yolo_detector()
        self.bridge = CvBridge()

    def callback(self, img):
        print("Received Image")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
            tick = datetime.datetime.now()
            out_frame = self.detector.detect(cv_image)
            tock = datetime.datetime.now()
            print("Handle time: {} seconds").format(tock-tick)
            out_image = self.bridge.cv2_to_imgmsg(out_frame, 'bgr8')
            self.publisher.publish()
        except CvBridgeError as error:
            print(error)
        #image = cv.CreateMat(1,1,CV_32F)
        
    def init(self):
        node = rospy.init_node("yolo_detection_node")
        rospy.Subscriber("/cam3d/rgb/image_rect_color", Image, self.callback)
        self.publisher = rospy.Publisher('yolo_detection', Image, queue_size=1)
        try: 
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting Down")
            cv.destroyAllWindows()

if __name__ == '__main__':
    node = yolo_detection_node()
    node.init()
