cmake_minimum_required(VERSION 3.5)
project(hydra_bringup)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/robot_description_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
