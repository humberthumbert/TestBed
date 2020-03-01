# ROS Installation Guide:

 - Install ROS melodic on Ubuntu 18.04, tutorial: http://wiki.ros.org/melodic/Installation/Ubuntu
 - Create a catkin workspace, tutorial: http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
 - Clone every package into the src folder of your catkin workspace
catkin_make command to build every package recursively

------------------------------------------------------

## Usage:

 - $ roslaunch velodyne_pointcloud VLP16_points.launch

This starts the node for the lidar sensor drivers for the lidar to start printing pointclouds on the topic /velodyne_points

 - $ roslaunch ipcamera_driver pixel1.launch
 - $ roslaunch ipcamera_driver all_pixel.launch

This starts the driver_node for the cameras to be able to stream images. pixel1 launches first phone, all_pixel launches all 4 phones

 - $ rosrun camera_node publisher.py

This python script starts the camera_node with the publisher script which handles all pixels

 - $ roslaunch but_calibration_camera_velodyne calibration_coarse.launch
 - $ roslaunch but_calibration_camera_velodyne calibration_fine.launch

Theese commands launch the calibration node coarse or fine.

------------------------------------------------------

## ROS commands: (see ROS tutorial as well in wiki)

 - $ roscore

Starts the master node running, which is needed by all ros nodes (most packages include this in their launchers so not always needed)

 - $ rosnode list

Prints all running nodes

 - $ rostopic list

Prints all the rostopics created by running nodes

 - $ rostopic echo /velodyne_points

Prints what is being written on a topic, in this example the topic for the velodyne point clouds is used.

 - $ rosrun rqt_graph rqt_graph

Graphs all the nodes and the topics running, needs an extension to work, just google the command to install it.