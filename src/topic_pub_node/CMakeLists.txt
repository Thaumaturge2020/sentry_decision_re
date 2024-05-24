cmake_minimum_required(VERSION 3.8)
project(topic_pub_node)

set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


if(NOT ROOT)
  set(ROOT ${PROJECT_SOURCE_DIR}/)
endif()

add_definitions(-DROOT="${ROOT}")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(backward_ros REQUIRED)
find_package(ament_cmake_auto REQUIRED)
include_directories("/usr/include/eigen3")
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()



add_library(topic_pub_node_1 SHARED src/topic_pub_1.cpp)
add_library(topic_pub_node_2 SHARED src/topic_pub_2.cpp)

target_include_directories(topic_pub_node_1 PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(topic_pub_node_2 PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(topic_pub_node_1 rclcpp rclcpp_action rclcpp_components std_msgs robot_msgs nav_msgs backward_ros)
ament_target_dependencies(topic_pub_node_2 rclcpp rclcpp_action rclcpp_components std_msgs robot_msgs nav_msgs backward_ros)


ament_auto_package(
  INSTALL_TO_SHARE
  launch
)

rclcpp_components_register_node(topic_pub_node_1 PLUGIN "topic_pub_node_1::TopicPublisher_1" EXECUTABLE topic_pub_node_main_1)
rclcpp_components_register_node(topic_pub_node_2 PLUGIN "topic_pub_node_2::TopicPublisher_2" EXECUTABLE topic_pub_node_main_2)

install(TARGETS
  topic_pub_node_1
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  topic_pub_node_2
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)