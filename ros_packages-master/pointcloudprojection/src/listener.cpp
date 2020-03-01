#include "ros/ros.h"
#include "std_msgs/String.h"
#include <opencv2/opencv.hpp>

#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include "sensor_msgs/Image.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/bind.hpp>
#include "image_projector.h"
#include "point_projector.h"

//Used for transformation
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
using namespace pcl;

// #include "const_data.h"
#include "cv_bridge/cv_bridge.h"

using namespace message_filters;
/*
Velodyne::Velodyne Velodyne::Velodyne::transform(float x, float y, float z, float rot_x, float rot_y, float rot_z)
{
  Eigen::Affine3f transf = getTransformation(x, y, z, rot_x, rot_y, rot_z);
  PointCloud<Point> new_cloud;
  transformPointCloud(point_cloud, new_cloud, transf);
  return Velodyne(new_cloud);
}
*/

cv::Mat image(1, 1, CV_32F);

ros::Publisher publish_images;
std::string input_topic;
std::string output_topic;

void convert_points(const sensor_msgs::PointCloud2 &msg) {
    // THE SIX DEGREES OF FREEDOM FROM THE CALIBRATION
    // THEY WILL AUTOMATICALLY BE CONVERTED TO THE TRANSFORMATION MATRIX
    // DON'T WORRY ABOUT ITT
    float six_dof[] = {0.0188819, -0.19959, -0.159947, 0.000680824, 0.00435048, -0.000106184};
    
    // THE INTRINSICS MATRIX
    float i_data[] = {253.5595, 0.0, 157.5681, 0.0,
	              0.0, 254.8321, 118.2755, 0.0,
                      0.0 ,0.0 , 1, 0.0};

    
    // EXTRACTION FROM THE GIVEN DATA
    float a = six_dof[3];
    float b = six_dof[4];
    float c = six_dof[5];

    // CALCULATION OF THE ROTATION MATRIX
    float r1 = cosf(b) * cosf(c);
    float r2 = -cosf(a) * sinf(c) + sinf(a) * sinf(b) * cos(c);
    float r3 = sinf(a) * sinf(c) + cosf(a) * sinf(b) * cosf(c);
    float r4 = cosf(b) * sinf(c);
    float r5 = cosf(a) * cosf(c) + sinf(a) * sinf(b) * sinf(c);
    float r6 = -sinf(a) * cosf(c) + cosf(a) * sinf(b) * sinf(c);
    float r7 = -sinf(b);
    float r8 = -sinf(a) * cosf(b);
    float r9 = cosf(a) * cosf(b);

    float r_data[] = {r1, r2, r3, 0,
                      r4, r5, r6, 0,
                      r7, r8, r9, 0};

    auto rotation_matrix = cv::Mat(3, 4, CV_32F, r_data);

    auto intrinsics_matrix = cv::Mat(3, 4, CV_32F, i_data);

    const float t_data[] = {six_dof[0], six_dof[1], six_dof[2]};
    
    auto translation = cv::Mat(3, 1, CV_32F);
    for (size_t i = 0; i < 3; i++)
        translation.at<float>(i) = *(t_data + i);

    point_projector p_projector{rotation_matrix, translation, intrinsics_matrix};
    if (image.rows == 1)
        return;
    
    Eigen::Affine3f transf;

    //Transform pointcloud
    if (input_topic.find('1') != std::string::npos){
        transf = getTransformation(0, 0, 0, M_PI/2, -M_PI/2, 0); //pixel1
    }    
    else if (input_topic.find('2') != std::string::npos){
        transf = getTransformation(0, 0, 0, M_PI/2, -M_PI, 0); //pixel2
    }
    else if (input_topic.find('3') != std::string::npos){
        transf = getTransformation(0, 0, 0, M_PI/2, -3*M_PI/2, 0); //pixel3
    }
    else if (input_topic.find('4') != std::string::npos){
        transf = getTransformation(0, 0, 0, M_PI/2, -2*M_PI, 0); //pixel4
    }
    //Eigen::Affine3f transf = getTransformation(0, 0, 0, M_PI/2, -3*M_PI/2, 0); //pixel3
    //Eigen::Affine3f transf = getTransformation(0, 0, 0, M_PI/2, -2*M_PI, 0); //pixel4
    PointCloud<pcl::PointXYZRGB> new_cloud, output_cloud;
    fromROSMsg(msg, new_cloud);
    transformPointCloud(new_cloud, output_cloud, transf);
    sensor_msgs::PointCloud2 msg2;
    toROSMsg(output_cloud, msg2);

    // sensor_msgs::PointCloud2 color_cloud2;
    // toROSMsg(pointcloud, color_cloud2);
    // color_cloud2.header = msg.header;

    auto result = std::vector < cv::Vec3f > {};
    auto in_x = sensor_msgs::PointCloud2ConstIterator<float>(msg2, "x");
    auto in_y = sensor_msgs::PointCloud2ConstIterator<float>(msg2, "y");
    auto in_z = sensor_msgs::PointCloud2ConstIterator<float>(msg2, "z");

    for (; in_x != in_x.end() && in_y != in_y.end() && in_z != in_z.end(); ++in_x, ++in_y, ++in_z) {
	//std::cout << *in_x << " " << *in_y << " " << *in_z << std::endl; //added
        result.push_back(cv::Vec3f(*in_x, *in_y, *in_z));
    }

    auto projected_points = p_projector.project_points(result);

    auto projector = image_projector{};
    auto mat = projector.project_image("/home/stamatis/my_ws/output.png", image, projected_points);

    cv_bridge::CvImage image_data;
    image_data.encoding = sensor_msgs::image_encodings::BGR8;
    image_data.image = mat;
    publish_images.publish(image_data);
}

void callback(const sensor_msgs::ImageConstPtr &img, const sensor_msgs::PointCloud2ConstPtr &msg) {
    //std::cout << "Received synchronized image and points" << std::endl;
    auto image_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    image_ptr->image.copyTo(image);
    assert(!image.empty());
    convert_points(*msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "image_projector");
    ros::NodeHandle n;
    input_topic = argv[1];    
    output_topic = argv[2];
    message_filters::Subscriber<sensor_msgs::Image> image_sub(n, input_topic, 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(n, "velodyne_points", 1);
    publish_images = n.advertise<sensor_msgs::Image>(output_topic, 1);

    typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, cloud_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));
    
    std::cout << "PointCloudProjector started" << std::endl;
    ros::spin();

    return 0;
}
