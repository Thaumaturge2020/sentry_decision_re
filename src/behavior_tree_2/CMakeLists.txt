cmake_minimum_required(VERSION 3.5)
project(behavior_tree_2)

#Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(NOT ROOT)
  set(ROOT ${PROJECT_SOURCE_DIR}/)
endif()

add_definitions(-DROOT="${ROOT}")

find_package(ament_cmake_auto REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(node_library REQUIRED)
find_package(robot_msgs REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rm_interfaces REQUIRED)
find_package(backward_ros REQUIRED)

add_executable(btree src/btree.cpp)

add_library(topic_transmit SHARED src/topic_transmit.cpp)

target_include_directories(btree PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

  target_include_directories(topic_transmit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(btree rclcpp std_msgs rclcpp_action rclcpp_components behaviortree_ros2 behaviortree_cpp node_library robot_msgs backward_ros rm_interfaces)

ament_target_dependencies(topic_transmit rclcpp std_msgs rclcpp_action rclcpp_components node_library robot_msgs backward_ros rm_interfaces)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)

rclcpp_components_register_node(topic_transmit PLUGIN "topic_transimit_node::TopicTransmitter" EXECUTABLE topic_transmit_node)


install(TARGETS
  btree
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  topic_transmit
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)