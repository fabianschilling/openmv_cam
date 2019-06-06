cmake_minimum_required(VERSION 2.8.3)
project(openmv_cam)

find_package(catkin REQUIRED COMPONENTS
  camera_info_manager_py
  cv_bridge
  rospy
  sensor_msgs
)
catkin_package()
