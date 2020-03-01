#!/usr/bin/env python
'''
Script to run PIXOR Detector on KITTI Raw Dataset
Generate a Series of BEV Predictions Images
'''

import numpy as np
import cv2
import sys
from cv_bridge import CvBridge, CvBridgeError
import os.path
import ctypes
import time
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import String
from model import PIXOR
from datetime import datetime
from pointcloud_detection import *


class pointcloud_detection_node:
    def __init__(self, config=None, cdll=False, isKitti=False):
        if config == None:
            config = {
                "ckpt_name": os.path.join(os.path.dirname(os.path.realpath(__file__)) ,"experiments/default/34epoch"),
                "use_bn": True,
                "cls_threshold": 0.5,
                "nms_iou_threshold": 0.1,
                "nms_top": 64,
                "geometry": {
                'L1': -40.0,
                'L2': 40.0,
                'W1': 0.0,
                'W2': 70.0,
                'H1': -2.5,
                'H2': 1.0,
                'grid_size': 0.1,
                'input_shape': [800, 700, 36],
                'label_shape': [200, 175, 7],
                },
            }

        
        self.pixor = pc_detector(config, cdll)
        self.bridge = CvBridge()

        rospy.init_node("pointcloud_detection_node")
        self.publisher = rospy.Publisher('pc_detection', Image, queue_size=1)
        rospy.Subscriber("velodyne_points", PointCloud2, self.callback)
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting Down")


    def callback(self, data):
        print("Process Point Cloud")
        points = pc2.read_points(data)
        points_array = []
        for point in points:
            points_array.append(point)

        points_array = np.array(points_array, dtype=np.float32)
        print("POINT_ARRAY", points_array)
        time, corners, scores, pred_bev = self.pixor(points_array, "")
        try:
            print("Publish BEV Image")
            self.publisher.publish(self.bridge.cv2_to_imgmsg(pred_bev, 'bgr8'))
        except CvBridgeError as e:
            print(e)


if __name__ == '__main__':
    node = pointcloud_detection_node(isKitti=False)
