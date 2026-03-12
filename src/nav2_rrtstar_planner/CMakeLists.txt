cmake_minimum_required(VERSION 3.8)
project(nav2_rrtstar_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_util REQUIRED)

# Add the C++ library
add_library(nav2_rrtstar_planner SHARED
  src/rrtstar_planner.cpp
)

target_include_directories(nav2_rrtstar_planner
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(nav2_rrtstar_planner
  rclcpp
  nav2_core
  nav2_costmap_2d
  geometry_msgs
  nav_msgs
  pluginlib
  tf2_ros
  nav2_util
)

# Export the plugin XML
pluginlib_export_plugin_description_file(nav2_core rrtstar_plugin.xml)

# Install rules
install(TARGETS nav2_rrtstar_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES rrtstar_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(nav2_rrtstar_planner)
ament_package()