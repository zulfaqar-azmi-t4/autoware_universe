cmake_minimum_required(VERSION 3.14)
project(autoware_collision_detector)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  SYSTEM
    ${EIGEN3_INCLUDE_DIR}
)

generate_parameter_library(collision_detector_node_parameters
  param/collision_detector_node_parameters.yaml
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/node.cpp
)

target_link_libraries(${PROJECT_NAME}
  collision_detector_node_parameters
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::collision_detector::CollisionDetectorNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
