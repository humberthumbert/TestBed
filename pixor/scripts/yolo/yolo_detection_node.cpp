#include <iostream>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"

#include "yolo_detector.h"

constexpr const bool show_result = false;

class yolo_detection_node {
    yolo_detector detector;

    size_t iterations = 0;
    size_t total_length = 0;

    decltype(std::chrono::steady_clock::now()) start;

    ros::Publisher pub_images;

public:
    yolo_detection_node()
            : start(std::chrono::steady_clock::now()) {
    }

    void callback(const sensor_msgs::Image img) {
        std::cout << "Received image" << std::endl;
        auto image_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
	cv::Mat image(1, 1, CV_32F);
        image_ptr->image.copyTo(image);

      	if (image.empty())
      	{
      		std::cout<< "Received empty image" << std::endl;
      		return;
      	}

        auto tick = std::chrono::steady_clock::now();
	auto output_frame = detector.detect(image);
        auto tock = std::chrono::steady_clock::now();
        total_length += std::chrono::duration_cast<std::chrono::milliseconds>(tock - tick).count();
        iterations++;
        auto fps = static_cast<double>(iterations) / std::chrono::duration_cast<std::chrono::seconds>(tock - start).count();
        std::cout << "Time per iteration: " << (total_length / iterations) << " (" << fps << " fps)" << std::endl;

	      cv_bridge::CvImage image_data;
	std::cout << "encoding" << std::endl;
        image_data.encoding = sensor_msgs::image_encodings::BGR8;
        image_data.image = output_frame;
	      pub_images.publish(image_data);
	
        if constexpr (show_result)
        {
    	      cv::imshow("Yolo Detection", output_frame);
    	      cv::waitKey(10);
        }
    }

    void init() {
        ros::NodeHandle n;
        
        ros::Subscriber sub_images = n.subscribe("cam3d/rgb/image_rect_color", 1, &yolo_detection_node::callback, this);
	pub_images = n.advertise<sensor_msgs::Image>("pixel1/yolo_detector_output", 1);
	//ros::Subscriber sub_images = n.subscribe("pixel1/image_rect_color", 1, &yolo_detection_node::callback, this);
	//      pub_images = n.advertise<sensor_msgs::Image>("pixel1/yolo_detector_output", 1);
        ros::spin();
    }

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "yolo_detector");
    yolo_detection_node node;
    node.init();

    return 0;
}
