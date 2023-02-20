cmake_minimum_required(VERSION 3.4)
project(hydra_hw)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
    controller_interface
    hardware_interface
    roscpp
    pluginlib
    urdf
    za_hw
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES za_hw
    CATKIN_DEPENDS
        controller_interface
        hardware_interface
        roscpp
        pluginlib
        urdf
        za_hw
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)