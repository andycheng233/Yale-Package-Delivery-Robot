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
  scripts/pedestrian_safety_node.py
  scripts/test_yolo.py
  scripts/test_depth.py
  scripts/set_initial_pose.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()