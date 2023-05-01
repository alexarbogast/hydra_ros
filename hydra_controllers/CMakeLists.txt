cmake_minimum_required(VERSION 3.4)
project(hydra_controllers)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  dynamic_reconfigure
  eigen_conversions
  za_hw
  za_msgs
  za_controllers
  hydra_hw
  geometry_msgs
  hardware_interface
  joint_limits_interface
  message_generation
  pluginlib
  realtime_tools
  roscpp
  urdf
)

find_package(Eigen3 REQUIRED)

add_service_files(
  DIRECTORY srv
  FILES SwitchCoordination.srv
)
generate_dynamic_reconfigure_options(
  cfg/hydra_param.cfg
)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES za_controllers
  CATKIN_DEPENDS
  controller_interface
  dynamic_reconfigure
  eigen_conversions
  za_hw
  za_msgs
  za_controllers
  hydra_hw
  geometry_msgs
  hardware_interface
  joint_limits_interface
  message_runtime
  pluginlib
  realtime_tools
  roscpp
  urdf
)

add_library(hydra_controllers
    src/hydra_controller.cpp
    src/control_methods.cpp
)
add_dependencies(hydra_controllers
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(hydra_controllers PUBLIC ${catkin_LIBRARIES})
target_include_directories(hydra_controllers SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_include_directories(hydra_controllers PUBLIC
  include
)

## Installation
install(TARGETS hydra_controllers
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(FILES hydra_controllers_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)