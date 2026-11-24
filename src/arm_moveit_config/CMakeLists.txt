cmake_minimum_required(VERSION 3.16)
project(arm_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_library(pick_place_panel SHARED src/pick_place_panel.cpp)
ament_target_dependencies(pick_place_panel ament_index_cpp pluginlib rviz_common)
target_link_libraries(pick_place_panel Qt5::Widgets)
pluginlib_export_plugin_description_file(rviz_common rviz/panels.xml)
install(TARGETS pick_place_panel LIBRARY DESTINATION lib)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/shelf_pick_place.py DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
