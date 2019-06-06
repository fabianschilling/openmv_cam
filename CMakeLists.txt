cmake_minimum_required(VERSION 2.8.3)
project(openmv_cam)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  camera_info_manager_py
)
catkin_package()
