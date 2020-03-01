#include <cstdlib>
#include <cstdio>
#include <math.h>
#include <algorithm>

#include "opencv2/opencv.hpp"

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl_ros/point_cloud.h>
#include <boost/foreach.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <velodyne_pointcloud/point_types.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

#include <but_calibration_camera_velodyne/Velodyne.h>
#include <but_calibration_camera_velodyne/Calibration.h>
#include <but_calibration_camera_velodyne/Calibration3DMarker.h>
#include <but_calibration_camera_velodyne/Image.h>

using namespace cv;
using namespace std;
using namespace ros;
using namespace message_filters;
using namespace pcl;
using namespace but_calibration_camera_velodyne;

string CAMERA_FRAME_TOPIC;
string CAMERA_INFO_TOPIC;
string VELODYNE_TOPIC;

// marker properties:
double STRAIGHT_DISTANCE; // 23cm
double RADIUS; // 8.25cm

int PIXEL = 0;

Mat projection_matrix;
Mat frame_rgb;
Velodyne::Velodyne pointcloud;
bool doRefinement = false;

bool writeAllInputs()
{
  bool result = true;

  pointcloud.save("velodyne_pc.pcd");
  cv::imwrite("frame_rgb.png", frame_rgb);
  cv::FileStorage fs_P("projection.yml", cv::FileStorage::WRITE);
  fs_P << "P" << projection_matrix;
  fs_P.release();

  // DEBUG
  ROS_INFO_STREAM("SAVED THE THINGS");

  return result;
}

Calibration6DoF calibration(bool doRefinement = false)
{

  Mat frame_gray;
  cvtColor(frame_rgb, frame_gray, CV_BGR2GRAY);

  // Marker detection:
  Calibration3DMarker marker(frame_gray, projection_matrix, pointcloud.getPointCloud(), STRAIGHT_DISTANCE, RADIUS);
  vector<float> radii2D;
  vector<Point2f> centers2D;
  ROS_INFO_STREAM("Start image detection");
  if (!marker.detectCirclesInImage(centers2D, radii2D))
  {
    // DEBUG	
    ROS_INFO_STREAM("FAILED TO DETECT CIRCLES IN IMAGE");
    return Calibration6DoF::wrong();
  }
  ROS_INFO_STREAM("DETECTED CIRCLES IN IMAGE");
  float radius2D = accumulate(radii2D.begin(), radii2D.end(), 0.0) / radii2D.size();

  vector<float> radii3D;
  vector<Point3f> centers3D;

  if (!marker.detectCirclesInPointCloud(centers3D, radii3D))
  {
    // DEBUG
    ROS_INFO_STREAM("FAILED TO DETECT CIRCLES IN POINTCLOUD");
    return Calibration6DoF::wrong();
  }
  // DEBUG	
  ROS_INFO_STREAM("DETECTED CIRCLES IN POINTCLOUD");
  
  float radius3D = accumulate(radii3D.begin(), radii3D.end(), 0.0) / radii3D.size();

  // rough calibration
  Calibration6DoF translation = Calibration::findTranslation(centers2D, centers3D, projection_matrix, radius2D,
                                                             radius3D);

  if (doRefinement)
  {
    ROS_INFO("Coarse calibration:");
    translation.print();
    ROS_INFO("Refinement process started - this may take a minute.");
    size_t divisions = 5;
    float distance_transl = 0.02;
    float distance_rot = 0.01;
    Calibration6DoF best_calibration, avg_calibration;
    Calibration::calibrationRefinement(Image::Image(frame_gray), pointcloud, projection_matrix, translation.DoF[0],
                                       translation.DoF[1], translation.DoF[2], distance_transl, distance_rot, divisions,
                                       best_calibration, avg_calibration);
    return avg_calibration;
  }
  else
  {
    return translation;
  }
}
void chatterCallback1(const sensor_msgs::ImageConstPtr& msg_img){
	ROS_INFO_STREAM("Image received at " << msg_img->header.stamp.toSec());
}
void chatterCallback2(const sensor_msgs::CameraInfoConstPtr& msg_img){
	ROS_INFO_STREAM("Info received at " << msg_img->header.stamp.toSec());
}
void chatterCallback3(const sensor_msgs::PointCloud2ConstPtr& msg_img){
	ROS_INFO_STREAM("Velodyne received at " << msg_img->header.stamp.toSec());
}
void callback(const sensor_msgs::ImageConstPtr& msg_img, const sensor_msgs::CameraInfoConstPtr& msg_info,
              const sensor_msgs::PointCloud2ConstPtr& msg_pc)
{

  ROS_INFO_STREAM("Image received at " << msg_img->header.stamp.toSec());
  ROS_INFO_STREAM( "Camera info received at " << msg_info->header.stamp.toSec());
  ROS_INFO_STREAM( "Velodyne scan received at " << msg_pc->header.stamp.toSec());

  // DEBUG
  ROS_INFO_STREAM("RECEIVED EVERYTHING");

  // Loading camera image:
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg_img, sensor_msgs::image_encodings::BGR8);
  frame_rgb = cv_ptr->image;
		
  // DEBUG	
  ROS_INFO_STREAM("LOADED CAMERA IMAGE");
  
  // Loading projection matrix:
  float p[12];
  float *pp = p;
  for (boost::array<double, 12ul>::const_iterator i = msg_info->P.begin(); i != msg_info->P.end(); i++)
  {
    *pp = (float)(*i);
    pp++;
  }
  cv::Mat(3, 4, CV_32FC1, &p).copyTo(projection_matrix);
	
  // DEBUG
  ROS_INFO_STREAM("LOADED PROJECTION MATRIX");

  // Loading Velodyne point cloud
  PointCloud<Velodyne::Point> pc;
  fromROSMsg(*msg_pc, pc);

  // DEBUG
  ROS_INFO_STREAM("LOADED POINTCLOUD");

  // x := x, y := -z, z := y,
  /*if (PIXEL == 0) {
    ROS_INFO_STREAM("Enter which phone you'd like to calibrate (1, 2, 3 or 4): ");
    cin >> PIXEL;
  }*/
  pcl::io::savePCDFile("/home/stamatis/my_ws/pointcl.pcd", pc);
  ROS_INFO_STREAM("Running for " << PIXEL);
  pointcloud = Velodyne::Velodyne(pc).transform(0, 0, 0, M_PI / 2, -M_PI / 2, 0);
  pcl::io::savePCDFile("/home/stamatis/my_ws/pointcloud.pcd", pointcloud.getPointCloud());
  /*switch(PIXEL) {
    case 1: {pointcloud = Velodyne::Velodyne(pc).transform(0, 0, 0, M_PI / 2, -M_PI / 2, 0);
pcl::io::savePCDFile("/home/stamatis/my_ws/pointcloud.pcd", pointcloud.getPointCloud());};

    case 2: pointcloud = Velodyne::Velodyne(pc).transform(0, 0, 0, M_PI / 2, M_PI, 0);

    case 3: pointcloud = Velodyne::Velodyne(pc).transform(0, 0, 0, M_PI / 2, M_PI / 2, 0);

    case 4: pointcloud = Velodyne::Velodyne(pc).transform(0, 0, 0, M_PI / 2, 0, 0);

    case 5: {
		Velodyne::Velodyne pointcloud1;
		Velodyne::Velodyne pointcloud2;
		Velodyne::Velodyne pointcloud3;
		Velodyne::Velodyne pointcloud4;

    		pointcloud1 = Velodyne::Velodyne(pc).transform(0, 0, 0, M_PI / 2, -M_PI / 2, 0);
    		pointcloud2 = Velodyne::Velodyne(pc).transform(0, 0, 0, M_PI / 2, M_PI, 0);
    		pointcloud3 = Velodyne::Velodyne(pc).transform(0, 0, 0, M_PI / 2, M_PI / 2, 0);
    		pointcloud4 = Velodyne::Velodyne(pc).transform(0, 0, 0, M_PI / 2, 0, 0);

		pcl::io::savePCDFile("/home/stamatis/my_ws/pointcloud1.pcd", pointcloud1.getPointCloud());
		pcl::io::savePCDFile("/home/stamatis/my_ws/pointcloud2.pcd", pointcloud2.getPointCloud());
		pcl::io::savePCDFile("/home/stamatis/my_ws/pointcloud3.pcd", pointcloud3.getPointCloud());
		pcl::io::savePCDFile("/home/stamatis/my_ws/pointcloud4.pcd", pointcloud4.getPointCloud());


    }
  }*/
  
  writeAllInputs();
  
  // DEBUG	
  ROS_INFO_STREAM("WROTE ALL INPUTS");

  Calibration6DoF calibrationParams = calibration(doRefinement);

  // DEBUG	
  ROS_INFO_STREAM("DID CALIBRATION");

  if (calibrationParams.isGood())
  {
    ROS_INFO_STREAM("Calibration succeeded, found parameters:");
    calibrationParams.print();
    shutdown();
  }
  else
  {
    ROS_WARN("Calibration failed - trying again after 5s ...");
    ros::Duration(5).sleep();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "calibration_node");

  int c;
  while ((c = getopt(argc, argv, "r")) != -1)
  {
    switch (c)
    {
      case 'r':
        doRefinement = true;
        break;
      default:
        return EXIT_FAILURE;
    }
  }

  ros::NodeHandle n;
  n.getParam("/but_calibration_camera_velodyne/camera_frame_topic", CAMERA_FRAME_TOPIC);
  n.getParam("/but_calibration_camera_velodyne/camera_info_topic", CAMERA_INFO_TOPIC);
  n.getParam("/but_calibration_camera_velodyne/velodyne_topic", VELODYNE_TOPIC);
  n.getParam("/but_calibration_camera_velodyne/marker/circles_distance", STRAIGHT_DISTANCE);
  n.getParam("/but_calibration_camera_velodyne/marker/circles_radius", RADIUS);

  message_filters::Subscriber<sensor_msgs::Image> image_sub(n, CAMERA_FRAME_TOPIC, 1);
  message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(n, CAMERA_INFO_TOPIC, 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(n, VELODYNE_TOPIC, 1);

  typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, info_sub, cloud_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3));

  ros::spin();

  return EXIT_SUCCESS;
}
