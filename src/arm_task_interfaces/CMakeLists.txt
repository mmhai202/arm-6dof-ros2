cmake_minimum_required(VERSION 3.16)
project(arm_task_interfaces)

# Humble's Python interface generator still uses FindPythonInterp/FindPythonLibs.
# Keep those modules available on CMake >= 3.27 and use ament's interpreter.
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()
find_package(ament_cmake REQUIRED)
set(PYTHON_EXECUTABLE "${Python3_EXECUTABLE}")
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} "action/PickPlace.action")
ament_export_dependencies(rosidl_default_runtime)
ament_package()
