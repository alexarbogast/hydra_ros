cmake_minimum_required(VERSION 3.4)
project(hydra_control)

find_package(catkin REQUIRED COMPONENTS
  ros_control_boilerplate 
  controller_interface
  controller_manager
  pluginlib
  realtime_tools
  za_control
  hydra_hw
  roscpp
  sensor_msgs
  tf2_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS
    ros_control_boilerplate
    controller_interface
    controller_manager
    pluginlib
    realtime_tools
    za_control
    hydra_hw
    roscpp
    sensor_msgs
    tf2_msgs
)

add_library(positioner_state_controller src/positioner_state_controller.cpp)
add_dependencies(positioner_state_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(positioner_state_controller ${catkin_LIBRARIES})
target_include_directories(positioner_state_controller SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
target_include_directories(positioner_state_controller PUBLIC include)

## Installation
install(TARGETS positioner_state_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES za_control_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)