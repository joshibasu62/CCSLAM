cmake_minimum_required(VERSION 3.8)
project(nav2_rrt_star_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_util REQUIRED)

# --- CHANGED: Renamed target to match the XML path 'nav2_rrt_star_planner' ---
add_library(nav2_rrt_star_planner src/rrt_star_planner.cpp)

target_include_directories(nav2_rrt_star_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Using ament_target_dependencies handles the linking and include directories automatically
ament_target_dependencies(nav2_rrt_star_planner
  nav2_core
  nav2_costmap_2d
  nav_msgs
  geometry_msgs
  pluginlib
  rclcpp
  tf2_ros
  nav2_util
)

# Export the XML file so pluginlib can find it
pluginlib_export_plugin_description_file(nav2_core rrt_star_planner_plugin.xml)

# --- CHANGED: Updated install target name ---
install(TARGETS nav2_rrt_star_planner
  EXPORT export_nav2_rrt_star_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES rrt_star_planner_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# --- CHANGED: Updated export target name ---
ament_export_libraries(nav2_rrt_star_planner)
ament_export_include_directories(include)
ament_package()