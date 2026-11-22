cmake_minimum_required(VERSION 3.16)
project(arm_planning)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/plan_joint_trajectory.py
  scripts/arm_planning_service.py
  scripts/call_planning_service.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
