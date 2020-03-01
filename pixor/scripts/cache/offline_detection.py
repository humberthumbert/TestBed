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
import message_filters
from sensor_msgs.msg import PointCloud2, Image
import sensor_msgs.point_cloud2 as pc2
from model import PIXOR
from postprocess import filter_pred, non_max_suppression
from utils import get_bev, plot_bev
from datetime import datetime
from yolo.yolo_detector import *


class pc_detector(object):

    def __init__(self, config, cdll):
        self.config = config
        self.cdll = cdll
        if self.cdll:
            self.LidarLib = ctypes.cdll.LoadLibrary('preprocess/LidarPreprocess.so')
        #self.device = torch.device('cuda:3' if torch.cuda.is_available() else 'cpu')
        self.device = torch.device('cpu')
        self.net = PIXOR(config['geometry'], config['use_bn']).to(self.device)

        self.net.set_decode(True)
        self.net.load_state_dict(torch.load(config['ckpt_name'], map_location=self.device))
        self.net.eval()

        for p in self.net.parameters():
            p.require_grad = False

        print("PIXOR BEV Detector Initialized!")

    def preprocess(self, velo, path):
        geom = self.config['geometry']
        velo_processed = np.zeros(geom['input_shape'], dtype=np.float32)
        if self.cdll:
            c_name = bytes(path).encode('utf-8')
            c_data = ctypes.c_void_p(velo_processed.ctypes.data)
            self.LidarLib.createTopViewMaps(c_data, c_name)
        else:
            def passthrough(velo):
                q = (geom['W1'] < velo[:, 0]) * (velo[:, 0] < geom['W2']) * \
                    (geom['L1'] < velo[:, 1]) * (velo[:, 1] < geom['L2']) * \
                    (geom['H1'] < velo[:, 2]) * (velo[:, 2] < geom['H2'])
                indices = np.where(q)[0]
                return velo[indices, :]
            velo = passthrough(velo)
        
            velo_processed = np.zeros(geom['input_shape'], dtype=np.float32)
            intensity_map_count = np.zeros((velo_processed.shape[0], velo_processed.shape[1]))
            for i in range(velo.shape[0]):
                x = int((velo[i, 1] - geom['L1']) / 0.1)
                y = int((velo[i, 0] - geom['W1']) / 0.1)
                z = int((velo[i, 2] - geom['H1']) / 0.1)
                velo_processed[x, y, z] = 1
                velo_processed[x, y, -1] += velo[i, 3]
                intensity_map_count[x, y] += 1
            velo_processed[:, :, -1] = np.divide(velo_processed[:, :, -1], intensity_map_count, \
                                                 where=intensity_map_count != 0)

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

        t_m = time.time()
        #print(pred)
        corners, scores = filter_pred(self.config, pred)
        input_np = bev.permute(1, 2, 0).cpu().numpy()

        t_post = time.time()
        pred_bev = get_bev(input_np, corners)

        t_s = [t_pre-t_start, t_m-t_pre, t_post-t_m]
        return t_s, corners, scores, pred_bev


def run(dataset, save_path, height=400):
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
    cdll = True
    pixor = pc_detector(config, cdll)
    yolo = yolo_detector()
    # Initialize path to Kitti dataset
    
    # Video Writer from OpenCV
    
    fourcc = cv2.VideoWriter_fourcc(*'MJPG') # Be sure to use lower case
    imshape = (config['geometry']['input_shape'][1] * 2, config['geometry']['input_shape'][0])
    videowriter = cv2.VideoWriter(save_path, fourcc, 10.0, imshape)

    avg_time = []
    for (item, velo_file) in enumerate(dataset.velo_files):
        velo = dataset.get_velo(item)
        path = dataset.velo_files[item]
        time, corners, scores, pred_bev = pixor(velo, path)
        #print(time)
        merged_im = np.zeros((pred_bev.shape[0], pred_bev.shape[1] * 2, 3), dtype=np.uint8)
        avg_time.append(time) 
        yolo_image = cv2.cvtColor(np.asarray(dataset.get_cam2(item), dtype=np.uint8), cv2.COLOR_RGB2BGR)
        image = yolo.detect(yolo_image)
        rows, cols = image.shape[:2]
        crop_x = int(merged_im.shape[0]//2 - height//2)
        crop_y = int(merged_im.shape[0]//2 + height//2)
        image = cv2.resize(image, (int(height/rows * cols), height))
        crop_i = int(image.shape[1]//2 - merged_im.shape[1]/4)
        crop_j = int(image.shape[1]//2 + merged_im.shape[1]/4) 
        merged_im[crop_x:crop_y, :pred_bev.shape[1], :] = image[:, crop_i:crop_j,:]
        merged_im[:, pred_bev.shape[1]:, :] = pred_bev
        videowriter.write(merged_im)        
    
    videowriter.release()
    avg_time = np.mean(avg_time, axis=0)
    print("Average Preprocessing Time:  {:.3f}s \n"
          "        Forward Time:        {:.3f}s \n"
          "        Postprocessing Time: {:.3f}s"
          .format(avg_time[0], avg_time[1], avg_time[2]))

def make_kitti_video(date, drive, basedir=None):
    # basedir = '/Users/charlie/Documents/Coursework/Group_Project/PIXOR/Kitti/'
    if basedir == None:
        basedir = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'data/')
    dataset = pykitti.raw(basedir, date, drive)
   
    videoname = "detection_{}_{}.avi".format(date, drive)
    save_path = os.path.join(basedir,  date, "{}_drive_{}_sync".format(date, drive), videoname)    
    run(dataset, save_path)
     
def generate_video():
    height = 400
    basedir = os.path.dirname(os.path.realpath(__file__))
    date = datetime.date(datetime.now())
    date = "_%s-%s-%s"%(date.year,date.month,date.day)
    
    videoname = "detection_image{}.avi".format(date)
    save_path = os.path.join(basedir, videoname)
    
    fourcc = cv2.VideoWriter_fourcc(*'MJPG')
    imshape = (1400, 800)
    videowriter = cv2.VideoWriter(save_path, fourcc, 10.0, imshape)
    for i in range(len(detected_images)):
        image = detected_images[i]
        cv2.imshow("Out", image)
        #pc = detected_pcs[i]
        #merged_im = np.zeros((pc.shape[0], pc.shape[1]*2, 3), dtype=np.uint8)
        #rows, cols = image.shape[:2]
        #crop_x = int(merged_im.shape[0]//2 - height//2)
        #crop_y = int(merged_im.shape[0]//2 + height//2)
        #image = cv2.resize(image, (int(height/rows*cols), height))
        #crop_i = int(image.shape[1]//2 - merged_im.shape[1]/4)
        #crop_j = int(image.shape[1]//2 + merged_im.shape[1]/4)
        #merged_im[crop_x:crop_y, :pc.shape[1]:, :] = image[:, crop_i:crop_j,:]
        #merged_im[:, pc.shape[1]:, :] = pc
        videowriter.write(image)
    videoname = "detection_pc{}.avi".format(date)
    save_path = os.path.join(basedir, videoname)
    fourcc = cv2.VideoWriter_fourcc(*'MJPG')
    imshape = (700,800)
    videowriter = cv2.VideoWriter(save_path, fourcc, 10.0, imshape)
    for i in range(len(detected_pcs)):
        pc = detected_pcs[i]
        videowriter.write(pc)

    videowriter.release()


def img_callback(img_data):
    # YOLO
    print("Recieved")
    height=400
    yolo = yolo_detector()
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(img_data, 'bgr8')
        detected_image = yolo.detect(cv_image)
        detected_images.append(detected_image)

    except CvBridgeError as error:
        print(error)

def pc_callback(pc_data):
    print("PC Received")
    # Convert velodyne point cloud to np.array data
    points = pc2.read_points(pc_data)
    points_array = []
    for point in points:
        points_array.append(point)
    points_array = np.array(points_array)
    
    # pass into run_realtime
    dirname = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'experiments/default/34epoch')
    config = {
      "ckpt_name": dirname,
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
    cdll = False
    pixor = pc_detector(config, cdll)

    # only show the Bird Eye View
    imshape = (config['geometry']['input_shape'][1]*2, config['geometry']['input_shape'][0])
    velo = points_array
    path = ""
    velo_2d = np.array([velo[:, 0], velo[:, 1]]).transpose()#.permute(1, 2, 0).cpu().numpy()
    #print(velo_2d)
    #bev = plot_bev(velo_2d)
    #cv2.imshow("BEV", bev)
    #cv2.waitKey(0)
    time, corners, scores, pred_bev = pixor(velo, path)
    detected_pcs.append(pred_bev)    


def make_rosbag_video():

    import rospy
    import yaml
    config = yaml.safe_load(open("config.yml"))
    velodyne_type = config['velodyne_type']
    image_type = config['image_type']
    rospy.init_node("Detection")
    rospy.Subscriber(velodyne_type, PointCloud2, pc_callback)
    rospy.Subscriber(image_type, Image, img_callback)
    #pc_submsg = message_filters.Subscriber(velodyne_type, PointCloud2)
    #img_submsg = message_filters.Subscriber(image_type, Image)
    #ts = message_filters.TimeSynchronizer([pc_submsg, img_submsg], 10)
    #ts.registerCallback(callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")
        cv2.destroyAllWindows()
    rospy.on_shutdown(generate_video)


detected_images = []
detected_pcs = []

if __name__ == '__main__':
    if sys.argv[1] == "kitti":
        date = sys.argv[2]
        driver = sys.argv[3]
        make_kitti_video(date, driver)
    elif sys.argv[1] == "rosbag":
        make_rosbag_video()
    elif sys.argv[1] == "rosbag":
        print("Nope")















