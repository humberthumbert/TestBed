#include "boost/filesystem.hpp"
#include "const_data.h"
#include "cv_bridge/cv_bridge.h"
#include "image_projector.h"
#include "pcl/conversions.h"
#include "point_cloud_loader_pcl.h"
#include "point_projector.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include "std_msgs/String.h"
#include <opencv2/opencv.hpp>

namespace fs = boost::filesystem;

class mock_sender {
  enum class State {
    WaitingBoth,
    WaitingImages,
    WaitingPoints,
    Paused,
    Streaming
  };

  fs::path images_path;
  fs::path points_path;

  ros::NodeHandle &node;
  ros::Publisher pub_images;
  ros::Publisher pub_clouds;

  State streaming_state = State::WaitingBoth;

  std::vector<fs::path> image_files;
  std::vector<fs::path> point_cloud_files;

  decltype(std::declval<std::vector<fs::path>>().begin()) it_image_files;
  decltype(std::declval<std::vector<fs::path>>().begin()) it_point_cloud_files;

  ros::Rate loop_rate;

public:
  mock_sender(ros::NodeHandle &n) : node(n), loop_rate(1) {}

  void init() {
    pub_images =
        node.advertise<sensor_msgs::Image>("pixel1/image_rect_color", 1000);
    pub_clouds =
        node.advertise<sensor_msgs::PointCloud2>("velodyne_points", 1000);

    ros::Subscriber sub_points = node.subscribe(
        "velodyne_points_path", 1, &mock_sender::callback_points_path, this);
    ros::Subscriber sub_images = node.subscribe(
        "image_path", 1, &mock_sender::callback_images_path, this);
  }

  void callback_points_path(std_msgs::String path) {
    if (streaming_state == State::WaitingBoth)
      streaming_state = State::WaitingImages;
    else if (streaming_state == State::WaitingPoints)
      streaming_state = State::Streaming;

    set_points_path(fs::path(path.data));
    load_point_cloud_files();
    it_point_cloud_files = point_cloud_files.begin();
  }

  void callback_images_path(std_msgs::String path) {
    if (streaming_state == State::WaitingBoth)
      streaming_state = State::WaitingPoints;
    else if (streaming_state == State::WaitingImages)
      streaming_state = State::Streaming;

    set_images_path(path.data);
    load_image_files();
    it_image_files = image_files.begin();
  }

  void set_images_path(fs::path path) { images_path = path; }

  void set_points_path(fs::path path) { points_path = path; }

  void load_image_files() { load_files({"png", "jpeg", "jpg"}, image_files); }

  void load_point_cloud_files() { load_files({"bin"}, point_cloud_files); }

  void load_files(std::vector<std::string> extensions,
                  std::vector<std::string> &output) {
    output.clear();
    for (auto it = fs::directory_iterator(images_path);
         it != fs::directory_iterator(); ++it) {

      auto path = (*it).path();
      if (extensions.find(path.extension()) == extensions.end()) {
        std::cout << "Skipping non-image file " << *it << "   "
                  << (*it).path().extension() << std::endl;
        continue;
      }
      output.push_back((*it).path());
    }
    std::sort(output.begin(), output.end());
  }

  void loop() {
    while (true) {
      switch (streaming_state) {
      case State::Paused:
        continue;
      case State::WaitingBoth:
      case State::WaitingImages:
      case State::WaitingPoints:
        continue;
      case State::Streaming:
        break;
      }

      if (point_cloud_files.empty()) {
        std::cout << "No point cloud to send" << std::endl;
        continue;
      }

      if (image_files.empty()) {
        std::cout << "No image to send" << std::endl;
        continue;
      }

      fs::path image_path = *it_image_files;
      fs::path point_cloud_path = *it_point_cloud_files;

      std::cout << image_path << "   " << point_cloud_path << std::endl;
      cv::Mat mat = cv::imread(image_path.string());
      assert(!mat.empty());

      cv_bridge::CvImage image_data;
      image_data.encoding = sensor_msgs::image_encodings::BGR8;
      image_data.image = mat;

      auto cloud = pcl_loader::load_from_file(point_cloud_path);
      sensor_msgs::PointCloud2 pc2;
      pcl::toROSMsg(cloud, pc2);

      pub_images.publish(image_data);
      pub_clouds.publish(pc2);
      ros::spinOnce();
      loop_rate.sleep();
      // std::cout << "Published, iteration no " << i << std::endl;

      ++it_image_files;
      ++it_point_cloud_files;
      // if (it == image_files.end())
      //   it = image_files.begin();
    }
  }
};

int main(int argc, char **argv) {
  std::cout << "Started" << std::endl;
  ros::init(argc, argv, "mock_sender");
  ros::NodeHandle n;

  mock_sender sender{n};
  sender.loop();

  return 0;
}
