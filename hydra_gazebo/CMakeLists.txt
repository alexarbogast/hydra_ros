cmake_minimum_required(VERSION 3.1.3)
project(hydra_gazebo)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
  roscpp
  gazebo_ros_control
  kdl_parser
  boost_sml
  control_toolbox
  hardware_interface
  joint_limits_interface
  eigen_conversions
  urdf
  hydra_control
  za_gazebo
  za_control
  hydra_controllers
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    gazebo_ros_control
    kdl_parser
    boost_sml
    control_toolbox
    hardware_interface
    joint_limits_interface
    eigen_conversions
    urdf
    hydra_control
    za_gazebo
    za_control
    hydra_controllers
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

include_directories(include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_library(hydra_hw_sim src/hydra_hw_sim.cpp src/positioner_hw_sim.cpp)
target_link_libraries(hydra_hw_sim ${catkin_LIBRARIES})
target_include_directories(hydra_hw_sim SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
add_dependencies(hydra_hw_sim ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(
  FILES hydra_hw_sim_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})