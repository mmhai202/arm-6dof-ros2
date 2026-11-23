cmake_minimum_required(VERSION 3.16)
project(arm_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch rviz worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
