cmake_minimum_required(VERSION 3.8)
project(yale_delivery_robot)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  worlds
  urdf
  launch
  config
  maps
  models
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/waypoint_ui_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME waypoint_ui_node
)

ament_package()