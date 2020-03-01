#include <iostream>
#include <chrono>

#include <opencv2/opencv.hpp>

#include <boost/filesystem.hpp>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"

#include "face_detector.h"

constexpr const bool show_results = false;

class face_detection_node {
    cascade_face_detector detector;

    size_t iterations = 0;
    size_t total_length = 0;

    decltype(std::chrono::steady_clock::now()) start;

    ros::Publisher pub_images;

public:
    face_detection_node(std::string face_cascade_file, std::string eyes_cascade_file)
            : detector(face_cascade_file, eyes_cascade_file), 
            start(std::chrono::steady_clock::now())
    {
    }

    void callback(const sensor_msgs::Image img) {
        //std::cout << "Received image" << std::endl;
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

        //std::cout << "Average time per iteration: " << (total_length / iterations) << " (" << fps << " fps)" << std::endl;

	      cv_bridge::CvImage image_data;
        image_data.encoding = sensor_msgs::image_encodings::BGR8;
        image_data.image = output_frame;
        image_data.header.stamp = img.header.stamp;
	      pub_images.publish(image_data);

        if constexpr (show_results)
        {
	          cv::imshow("Face Detection", output_frame);
	          cv::waitKey(10);
        }
    }

    void init(std::string input_topic, std::string output_topic) {
        ros::NodeHandle n;
        ros::Subscriber sub_images = n.subscribe(input_topic, 1, &face_detection_node::callback, this);
	pub_images = n.advertise<sensor_msgs::Image>(output_topic, 1);
        std::cout << "FaceDetectionNode started" << std::endl;
	ros::spin();
    }

};

int main(int argc, char **argv) {
    if (argc < 3)
    {
        std::cerr << "Usage: face_detection_node <face_cascade_file> <eyes_cascade_file>" << std::endl;
        return 2;
    }

    std::string faces_file = argv[1];
    std::string eyes_file = argv[2];

    ros::init(argc, argv, "face_detection_node");
    face_detection_node node {faces_file, eyes_file};
    node.init(argv[3], argv[4]);

    return 0;
}
