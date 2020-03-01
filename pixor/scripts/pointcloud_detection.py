#!/usr/bin/env python
'''
Script to run PIXOR Detector on KITTI Raw Dataset
Generate a Series of BEV Predictions Images
'''

import torch
import numpy as np
import pykitti
import os.path
import ctypes
import time
from cv_bridge import CvBridge
from model import PIXOR
from postprocess import filter_pred, non_max_suppression
from utils import get_bev, plot_bev
import rospy
from sensor_msgs.msg import PointCloud2, Image
from numpy.ctypeslib import ndpointer

class pc_detector(object):

    def __init__(self, config, cdll):
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

        self.config = config
        self.cdll = cdll
        path = os.path.dirname(os.path.abspath(__file__))
        path = os.path.join(path, 'preprocess/LidarPreprocess.so')
        print("PATH:=", path)
        #if self.cdll:
        self.LidarLib = ctypes.cdll.LoadLibrary(path)
        #self.device = torch.device('cuda:3' if torch.cuda.is_available() else 'cpu')
        print("AFTER LOADLIBRARY")
        self.device = torch.device('cpu')
        self.net = PIXOR(config['geometry'], config['use_bn']).to(self.device)
        print(config['ckpt_name'])
        self.net.set_decode(True)
        self.net.load_state_dict(torch.load(config['ckpt_name'], map_location=self.device))
        self.net.eval()

        for p in self.net.parameters():
            p.require_grad = False
        self.bridge = CvBridge()
        print("PIXOR BEV Detector Initialized!")
   
   
    def init(self):
        rospy.init_node("pc_detector")
        self.publisher = rospy.Publisher('pc_detection', Image, queue_size=1)
        rospy.Subscriber("/velodyne/velodyne_points", PointCloud2, self.callback)
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting Down")


    def preprocess(self, velo, path):
        geom = self.config['geometry']
        velo_processed = np.zeros(geom['input_shape'], dtype=np.float32)
        if self.cdll:
            c_name = bytes(path).encode('utf-8')
            c_data = ctypes.c_void_p(velo_processed.ctypes.data)
            self.LidarLib.createTopViewMaps(c_data, c_name)
        else:
            func = self.LidarLib.createTopViewMaps2
            func.restype = None
            c_float_p = ctypes.POINTER(ctypes.c_float)
            func.argtypes = [ctypes.c_void_p, c_float_p, ctypes.c_int]
            velo = velo.astype(np.float32)
            func(velo_processed.ctypes.data, velo.ctypes.data_as(c_float_p), velo.shape[0])

        #    def passthrough(velo):
        #        q = (geom['W1'] < velo[:, 0]) * (velo[:, 0] < geom['W2']) * \
        #            (geom['L1'] < velo[:, 1]) * (velo[:, 1] < geom['L2']) * \
        #            (geom['H1'] < velo[:, 2]) * (velo[:, 2] < geom['H2'])
        #        indices = np.where(q)[0]
        #        return velo[indices, :]
        #    velo = passthrough(velo)
        #
        #    velo_processed = np.zeros(geom['input_shape'], dtype=np.float32)
        #    intensity_map_count = np.zeros((velo_processed.shape[0], velo_processed.shape[1]), dtype=np.float32)
        #    for i in range(velo.shape[0]):
        #        x = int((velo[i, 1] - geom['L1']) / 0.1)
        #        y = int((velo[i, 0] - geom['W1']) / 0.1)
        #        z = int((velo[i, 2] - geom['H1']) / 0.1)
        #        velo_processed[x, y, z] = 1
        #        velo_processed[x, y, -1] += velo[i, 3]
        #        intensity_map_count[x, y] += 1.0
        #    velo_processed[:, :, -1] = np.divide(velo_processed[:, :, -1], intensity_map_count, \
        #                                         where=intensity_map_count != 0)
        #    print(np.nonzero(velo_processed).shape)
        #    velo_precessed[velo_precessed < 1e-4] = 0.0
        #    print(np.nonzero(velo_processed).shape)
        velo_processed = torch.from_numpy(velo_processed).permute(2, 0, 1).to(self.device)
        velo_processed.require_grad=False
        return velo_processed

    def postprocess(self, pred):
        cls_pred = pred[..., 0]
        activation = cls_pred > self.config['cls_threshold']
        num_boxes = int(activation.sum())

        if num_boxes == 0:
            print("No bounding box found")
            return [], []

        corners = torch.zeros((num_boxes, 8))
        for i in range(1, 9):
            corners[:, i - 1] = torch.masked_select(pred[i, ...], activation)
        corners = corners.view(-1, 4, 2).numpy()
        scores = torch.masked_select(cls_pred, activation).cpu().numpy()

        # NMS
        #print(corners)
        selected_ids = non_max_suppression(corners, scores, self.config['nms_iou_threshold'])
        corners = corners[selected_ids]
        scores = scores[selected_ids]

        return corners, scores

    def __call__(self, velo, path):
        t_start = time.time()
        bev = self.preprocess(velo, path)
        t_pre = time.time()
        with torch.no_grad():
            pred = self.net(bev.unsqueeze(0)).squeeze_(0)
        print("Predicition:", pred) 
        t_m = time.time()
        corners, scores = filter_pred(self.config, pred)
        input_np = bev.permute(1, 2, 0).cpu().numpy()

        t_post = time.time()
        pred_bev = get_bev(input_np, corners)

        t_s = [t_pre-t_start, t_m-t_pre, t_post-t_m]
        return t_s, corners, scores, pred_bev


    def callback(self, data):
        print("Process Point Cloud")
        time, corners, scores, pred_bev = self(data, "")
        try:
            print("Publish BEV Image")
            self.publisher.publish(self.bridge.cv2_to_imgmsg(pred_bev), 'bgr8')
        except CvBridgeError as e:
            print(e)
       

