#!/usr/bin/env python 
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import String
import pykitti
import message_filters
from pointcloud_detection_node import *
from yolo.yolo_detection_node import *
import os
import re
import numpy as np
import sys

calibration_files = ['calib_cam_to_cam.txt', 'calib_imu_to_velo.txt', 'calib_imu_to_velo.txt']
def xyz_array_to_pointcloud2(points, stamp=None, frame_id=None):
    '''
    Create a sensor_msgs.PointCloud2 from an array of points.
    '''
    msg = PointCloud2()
    if stamp:
        msg.header.stamp = stamp
    if frame_id:
        msg.header.frame_id = frame_id
    if len(points.shape) == 3:
        msg.height = points.shape[1]
        msg.width = points.shape[0]
    else:
        msg.height = 1
        msg.width = len(points)
        msg.fields = [
                PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
                PointField('rgba', 12, PointField.FLOAT32, 1)]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 12*points.shape[0]
        msg.is_dense = int(np.isfinite(points).all())
        msg.data = np.asarray(points, np.float32).tostring()
    return msg

class mock_sender:
    def __init__(self, *args, **kwargs):
        """
        accept kwargs:
        kitti_dir: format 20xx_xx_xx
        kitti_drive: string xxxx
        image_dir: string path
        image_dirs: string path
        pc_dir: string path
        pc_dirs: string path
        rosbag: string rosbag
        """    
        if kwargs['kitti_dir'] != None:
            if kwargs['kitti_drive'] == None:
                assert("Please enter the corresponding drive number. kitti_drive=xxxx ")
            path = kwargs['kitti_dir']
            date = path.split('/')[-1]
            path = path[:-len(date)]
            drive = kwargs['kitti_drive']
            for f in calibration_files:
                if f not in os.listdir(path):
                    assert('Missing Calibration Files, Please download and place in directory.')
            self.dataset = pykitti.raw(path, date, drive)
        
        elif ((kwargs['image_dir'] != None) or (kwargs['image_dirs'] != None)) and\
                ((kwargs['pc_dir'] != None) or (kwargs['pc_dirs'] != None)):  
            self.image_files = []
            if kwargs['image_dir'] != None:
                image_dir = kwargs['image_dir']
                for item in os.listdir(image_dir):
                    if item.endswith(('.png', '.jpeg', 'jpg')):
                        self.image_files.append(os.path.join(image_dir, item))
            elif kwargs['image_dirs'] != None:
                image_dirs = kwargs['image_dirs']
                for subdir in os.listdir(image_dirs):
                    if os.path.isdir(subdir):
                        for item in os.listdir(subdir):
                            if item.endswith(('.png', '.jpeg', 'jpg')):
                                self.image_files.append(os.path.join(image_dir, subdir, item))
            
            self.pc_files = []
            if kwargs['pc_dir'] != None:
                pc_dir = kwargs['pc_dir']
                for item in os.listdir(pc_dir):
                    if '.bin' in item:
                        self.pc_files.append(os.path.join(pc_dir, item))
            elif kwargs['pc_dirs'] != None:
                pc_dirs = kwargs['pc_dirs']
                for subdir in os.listdir(pc_dirs):
                    if os.path.isdir(subdir):
                        for item in os.listdir(subdir):
                            if '.bin' in item:
                                self.pc_files.append(os.path.join(pc_dirs, subdir, item))

        else:
            self.rosbag = kwargs['rosbag']
        
        rospy.init_node("MockSender")
        self.pc_publisher = rospy.Publisher('velodyne_points', PointCloud2, queue_size = 10)
        self.pc_path_publisher = rospy.Publisher('kitti_path', String, queue_size = 1)
        self.image_publisher = rospy.Publisher('/cam3d/rgb/image_rect_color', Image, queue_size = 10)
        self.bridge = CvBridge()
        self.run()

    def run(self):
        if self.dataset != None:
            for i in range(len(self.dataset)):
                pointcloud = self.dataset.get_velo(i)
                pointcloud_path = self.dataset.velo_files[i]
                #pc = pc2.read_points(pointcloud) 
                msg_pc = xyz_array_to_pointcloud2(pointcloud)
                image = self.dataset.get_cam2(i)
                image = cv2.cvtColor(np.asarray(image, dtype=np.uint8), cv2.COLOR_RGB2BGR)
                msg_image = self.bridge.cv2_to_imgmsg(image, 'bgr8')
                self.pc_publisher.publish(msg_pc)
                self.image_publisher.publish(msg_image)
                print("Kitti RAW:", pointcloud)
                print("--------------------------------------------------------------------")
        elif (self.image_files != None) and (self.pc_files != None):
            assert(len(self.pc_files) != len(self.image_files))
            for i in len(self.pc_files):
                pc_path = self.pc_files[i]
                image_path = self.image_files[i]
                pc = np.fromfile(path, dtype=np.float32, count=-1).reshape([-1,4])
                image = cv2.imread(image_path)        
                msg_image = self.bridge.cv2_to_imgmsg(image, 'bgr8')
                self.pc_publisher.publish(pc)
                self.image_publisher.publish(msg_image)
        else:
            print('rosbag')

if __name__ == '__main__':
    argv = {}
    i = 1
    for i in range(1,8):
        arg = sys.argv[i]
        rospy.loginfo(arg)
        if arg.split(':')[1] != 'n':
            argv[arg.split(":")[0]] = arg.split(':')[1]
        else:
            argv[arg.split(':')[0]] = None
    print(argv)
    sender = mock_sender(kitti_dir=argv["kitti_dir"], kitti_drive=argv["kitti_drive"], image_dir=argv["image_dir"], image_dirs=argv["image_dirs"], pc_dir=argv["pc_dir"], pc_dirs=argv["pc_dirs"], rosbag=argv["bag"])
