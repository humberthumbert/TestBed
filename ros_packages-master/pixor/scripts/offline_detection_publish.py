#!/usr/bin/env python
'''
Script to run PIXOR Detector on KITTI Raw Dataset
Generate a Series of BEV Predictions Images
'''

import torch
import numpy as np
import pykitti
import cv2
import sys
from cv_bridge import CvBridge, CvBridgeError
import os.path
import ctypes
import time
import rospy
import rosbag
import message_filters
from sensor_msgs.msg import PointCloud2, Image
import sensor_msgs.point_cloud2 as pc2
from model import PIXOR
from postprocess import filter_pred, non_max_suppression
from utils import get_bev, plot_bev
from datetime import datetime
from yolo.yolo_detector import *
from pointcloud_detection import *
import re

class offline_pipeline:
    def __init__(self, camera0=False, camera1=False, camera2=False, camera3=False, pc_front=False):
        self.bridge = CvBridge()
        self.pc_publishers = {}
        self.image_publishers = {}
        if camera0:
            self.image_publishers[0] = rospy.Publisher('image_detection0', Image, queue_size=10)
        if camera1:
            self.image_publishers[1] = rospy.Publisher('image_detection1', Image, queue_size=10)
        if camera2:
            self.image_publishers[2] = rospy.Publisher('image_detection2', Image, queue_size=10)
        if camera3:
            self.image_publishers[3] = rospy.Publisher('image_detection3', Image, queue_size=10)
        if pc_front:
            self.pc_publishers[0] = rospy.Publisher('pc_detection0', Image, queue_size=10)
         
        rospy.init_node("offline_pipeline")
    
    
    def run(self, dataset, cdll, height=400):
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
        # Initialize Detector
        pixor = pc_detector(config, cdll)
        yolo = yolo_detector()

        # Initialize path to Kitti dataset        
        for (item, velo_file) in enumerate(dataset.velo_files):
            for publisher in self.pc_publishers.values():
                velo = dataset.get_velo(item)
                path = dataset.velo_files[item]
                time, corners, scores, pred_bev = pixor(velo, path)
                #cv2.imshow('output', pred_bev)
                #cv2.waitKey(0)
                out_image = self.bridge.cv2_to_imgmsg(pred_bev, 'bgr8')
                publisher.publish(out_image)
            
            for key in self.image_publishers.keys():
                yolo_image = None
                if key == 0:
                    yolo_image = cv2.cvtColor(np.asarray(dataset.get_cam0(item), dtype=np.uint8), cv2.COLOR_RGB2BGR)
                elif key == 1:
                    yolo_image = cv2.cvtColor(np.asarray(dataset.get_cam1(item), dtype=np.uint8), cv2.COLOR_RGB2BGR)
                elif key == 2:
                    yolo_image = cv2.cvtColor(np.asarray(dataset.get_cam2(item), dtype=np.uint8), cv2.COLOR_RGB2BGR)
                elif key == 3:
                    yolo_image = cv2.cvtColor(np.asarray(dataset.get_cam3(item), dtype=np.uint8), cv2.COLOR_RGB2BGR)
                
                publisher = self.image_publishers[key]
                image = yolo.detect(yolo_image)
                out_image = CvBridge().cv2_to_imgmsg(image, 'bgr8')
                publisher.publish(out_image)
        print("Publish Done")            


    def make_kitti_stream(self, path, date, drive, basedir=None):
        if not os.path.isdir(path):
            print("PATH NOT EXISTS")
            return
        dataset = pykitti.raw(path, date, drive)
        self.run(dataset, cdll=True)
         

    def make_rosbag_stream(self, path):
        dataset = kitti_generator()
        for topic, msg, t in rosbag.Bag(path).read_messages(topics=['/velodyne_points', '/cam3d/rgb/image_rect_color']):
            print(t)
            if topic == '/velodyne_points':
                points = pc2.read_points(msg)
                pc = []
                for point in points:
                    pc.append(point)
                pc = np.array(pc, dtype=np.float32)
                pc = np.delete(pc, -1, axis=1)
                dataset.input(pointcloud=pc)
            if topic == '/cam3d/rgb/image_rect_color':
                image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
                image = np.array(image)
                dataset.input(cam2=image)
        self.run(dataset, cdll=False)


class kitti_generator():
    def __init__(self):
        self.velo_data = []
        self.velo_files = []
        self.camN_files = [[],[],[],[]]
    
    def input(self, pointcloud=np.array([]), cam0=np.array([]), cam1=np.array([]), cam2=np.array([]), cam3=np.array([])):
        if len(pointcloud) != 0:
            self.velo_data.append(pointcloud)
            self.velo_files.append("")
        elif len(cam0) != 0:
            self.camN_files[0].append(cam0)
        elif len(cam1) != 0:
            self.camN_files[1].append(cam1)
        elif len(cam2) != 0:
            self.camN_files[2].append(cam2)
        elif len(cam3) != 0:
            self.camN_files[3].append(cam3)

    def get_velo(self, idx):
        return self.velo_data[idx]
    def get_cam0(self, idx):
        return self.camN_files[0][idx]
    def get_cam1(self, idx):
        return self.camN_files[1][idx]
    def get_cam2(self, idx):
        return self.camN_files[2][idx]
    def get_cam3(self, idx):
        return self.camN_files[3][idx]

if __name__ == '__main__':
    if sys.argv[1] == "-kitti":
        path = sys.argv[2]
        date = ""
        drive = ""
        if os.path.isdir(path):
            for item in os.listdir(path):
                pattern_date = re.compile("[0-9]{4}_[0-9]{2}_[0-9]{2}")
                if pattern_date.match(item):
                    date = item
                    for subitem in os.listdir(os.path.join(path, item)):
                        pattern_drive = re.compile("[0-9]{4}_[0-9]{2}_[0-9]{2}_drive_[0-9]{4}_sync")
                        if pattern_drive.match(subitem):
                            drive = subitem.split('_')[-2]
                            break
        #print(date, drive)
        pc_front = False if sys.argv[3] == '0' else True 
        camera0 = False if sys.argv[4] == '0' else True 
        camera1 = False if sys.argv[5] == '0' else True 
        camera2 = False if sys.argv[6] == '0' else True 
        camera3 = False if sys.argv[7] == '0' else True
        pipeline = offline_pipeline(camera0=camera0, camera1=camera1, camera2=camera2, camera3=camera3, pc_front=pc_front)
        pipeline.make_kitti_stream(path, date, drive)

    elif sys.argv[1] == "-custom":
       1 
    elif sys.argv[1] == "-rosbag":
        path = sys.argv[2]
        
        pc_front = False if sys.argv[3] == '0' else True 
        camera0 = False if sys.argv[4] == '0' else True 
        camera1 = False if sys.argv[5] == '0' else True 
        camera2 = False if sys.argv[6] == '0' else True 
        camera3 = False if sys.argv[7] == '0' else True
        pipeline = offline_pipeline(camera0=camera0, camera1=camera1, camera2=camera2, camera3=camera3, pc_front=pc_front)
        
        pipeline.make_rosbag_stream(path)
        



