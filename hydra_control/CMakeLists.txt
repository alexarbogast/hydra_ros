cmake_minimum_required(VERSION 3.4)
project(hydra_control)

find_package(catkin REQUIRED COMPONENTS
  ros_control_boilerplate 
  controller_interface
  controller_manager
  roscpp
  tf2_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS
    ros_control_boilerplate
    controller_interface
    controller_manager
    roscpp
    tf2_msgs
)
