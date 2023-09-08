cmake_minimum_required(VERSION 3.4)
project(hydra_bringup)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY config launch rviz
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
