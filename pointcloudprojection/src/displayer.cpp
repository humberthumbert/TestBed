#include "cv_bridge/cv_bridge.h"
#include "image_projector.h"
#include "point_projector.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include "std_msgs/String.h"
#include <opencv2/opencv.hpp>
#include <string>

//evaluation start

//#include <chrono>
//#include <ctime>

//evaluation end
cv::Mat image(1, 1, CV_32F);
std::string window_name;

void callback_images(const sensor_msgs::Image &img, std::string window_name) {
  static bool created = false;
  if (!created) {
    cv::namedWindow(window_name, cv::WINDOW_NORMAL);
    created = true;
  }
  auto image_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
  image_ptr->image.copyTo(image);

  cv::imshow(window_name, image);

  //evaluation start

  //using namespace std::chrono;
  //milliseconds ms = duration_cast <milliseconds>(
  //	system_clock::now().time_since_epoch()
  //);
  //std::cout <<ms.count() <<"\n";

  //evaluation end
  cv::waitKey(1);
}

void callback1(const sensor_msgs::Image &img) {
  callback_images(img, window_name);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "image_displayer");
  ros::NodeHandle n;
  std::string video_topic;
  video_topic = argv[1];
  window_name = argv[2];

  ros::Subscriber sub_images = n.subscribe(video_topic, 1, callback1);

  ros::spin();

  return 0;
}
