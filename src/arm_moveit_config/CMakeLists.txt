cmake_minimum_required(VERSION 3.16)
project(arm_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(arm_task_interfaces REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(yaml-cpp REQUIRED)

add_library(pick_place_panel SHARED src/pick_place_panel.cpp)
ament_target_dependencies(pick_place_panel ament_index_cpp pluginlib rviz_common
  rclcpp rclcpp_action arm_task_interfaces)
target_link_libraries(pick_place_panel Qt5::Widgets yaml-cpp)
pluginlib_export_plugin_description_file(rviz_common rviz/panels.xml)
install(TARGETS pick_place_panel LIBRARY DESTINATION lib)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/shelf_pick_place.py scripts/pick_place_server.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  ament_add_test(workspace_unit_tests
    COMMAND "${Python3_EXECUTABLE}" -m unittest discover
      -s "${CMAKE_CURRENT_SOURCE_DIR}/../../tests" -v
    ENV PYTHONDONTWRITEBYTECODE=1 ARM_RUN_MOVEIT_TESTS=0
      ARM_RUN_GAZEBO_TESTS=0 ARM_RUN_GAZEBO_GRASP_TESTS=0 ARM_RUN_TASK_ACTION_TESTS=0
    TIMEOUT 60
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )
endif()

ament_package()
