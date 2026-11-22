cmake_minimum_required(VERSION 3.16)
project(arm_description)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY urdf meshes
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/prepare_ur7e.py
  scripts/colorize_ur7e.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
