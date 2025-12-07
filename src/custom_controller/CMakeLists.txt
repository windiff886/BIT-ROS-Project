cmake_minimum_required(VERSION 3.5)
project(custom_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(angles REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_util
  nav2_costmap_2d
  nav_msgs
  geometry_msgs
  pluginlib
  tf2_ros
  tf2_geometry_msgs
  tf2
  angles
)

include_directories(include)

add_library(stanley_controller SHARED src/stanley_controller.cpp)
target_include_directories(stanley_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(stanley_controller ${dependencies})

pluginlib_export_plugin_description_file(nav2_core plugin.xml)

install(TARGETS stanley_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/
  DESTINATION include)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(${dependencies})
ament_export_include_directories(include)
ament_export_libraries(stanley_controller)

ament_package()
