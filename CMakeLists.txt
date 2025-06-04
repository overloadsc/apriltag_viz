cmake_minimum_required(VERSION 3.5)
project(apriltag_viz)

set(CMAKE_CXX_STANDARD 14)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)

add_library(apriltag_viz SHARED src/apriltag_viz.cpp)
ament_target_dependencies(apriltag_viz
  rclcpp
  rclcpp_components
  image_transport
  cv_bridge
  sensor_msgs
  tf2_ros
  apriltag_msgs
  geometry_msgs
  OpenCV
  tf2_msgs)
rclcpp_components_register_nodes(apriltag_viz "AprilVizNode")

install(TARGETS apriltag_viz
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
