cmake_minimum_required(VERSION 3.10)
project(face_detection_node)

catkin_package()

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS PCL cv_bridge roscpp rospy std_msgs genmsg)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(face_detection_node src/face_detection_node.cpp)
target_link_libraries(face_detection_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(face_detector_tests src/face_detector_tests.cpp)
target_link_libraries(face_detector_tests ${catkin_LIBRARIES} ${OpenCV_LIBS})
