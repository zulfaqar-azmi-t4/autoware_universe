cmake_minimum_required(VERSION 3.14)
project(autoware_manual_trajectory)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::manual_trajectory::ManualTrajectory"
  EXECUTABLE manual_trajectory
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
