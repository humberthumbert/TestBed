cmake_minimum_required(VERSION 3.10)
project(yolo_detection_node)

catkin_package()

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS PCL cv_bridge roscpp rospy std_msgs genmsg)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(yolo_detection_node src/yolo_detection_node.cpp)
target_link_libraries(yolo_detection_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
