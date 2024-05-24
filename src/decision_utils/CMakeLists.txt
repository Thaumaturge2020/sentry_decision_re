cmake_minimum_required(VERSION 3.8)
project(decision_utils)

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
find_package(OpenCV 4.5.4 REQUIRED)
find_package(backward_ros REQUIRED)
include_directories("/usr/include/eigen3")

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_components
  std_msgs
  OpenCV
)

aux_source_directory(src/. plugin_cpp_list)

add_library(decision_utils SHARED ${plugin_cpp_list})


target_include_directories(decision_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  
list(APPEND plugin_libs decision_utils)


ament_target_dependencies(decision_utils rclcpp rclcpp_action rclcpp_components std_msgs OpenCV backward_ros)

# foreach(node_plugin ${plugin_libs})
#   ament_target_dependencies(${node_plugin} ${dependencies})
#   target_compile_definitions(${node_plugin} PRIVATE BT_PLUGIN_EXPORT)
# endforeach()


install(TARGETS
  decision_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(${PROJECT_SOURCE_DIR}/include)

ament_export_libraries(${plugin_libs})

ament_export_dependencies(${dependencies})

#INSTALL( DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
#    DESTINATION ${BTNODE_INCLUDE_DESTINATION}
#    FILES_MATCHING PATTERN "*.h*")

ament_package()
