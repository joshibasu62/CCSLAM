cmake_minimum_required(VERSION 3.8)
project(map_merge_3d)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ========================================
# 1. FIND DEPENDENCIES
# ========================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

# Find PCL (Point Cloud Library) - System Dependency
find_package(PCL REQUIRED COMPONENTS core features filters io registration visualization)
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

# ========================================
# 2. INCLUDE DIRECTORIES
# ========================================
include_directories(
  include
  src
  ${PCL_INCLUDE_DIRS}
)

# ========================================
# 3. BUILD SHARED LIBRARY (Core Logic)
# ========================================
# We build the heavy math logic as a library first
add_library(map_merging
  src/features.cpp
  src/graph.cpp
  src/map_merging.cpp
  src/matching.cpp
)

# Link ROS 2 deps to the library
ament_target_dependencies(map_merging
  rclcpp
  sensor_msgs
  pcl_conversions
  tf2
  tf2_eigen
  tf2_geometry_msgs
)
# Link PCL to the library
target_link_libraries(map_merging ${PCL_LIBRARIES})


# ========================================
# 4. BUILD NODE (The Executable)
# ========================================
add_executable(map_merge_node
  src/map_merge_node.cpp
)

# Link the node to the library we just built above
target_link_libraries(map_merge_node map_merging)

# Link ROS 2 deps to the node
ament_target_dependencies(map_merge_node
  rclcpp
  sensor_msgs
  pcl_conversions
  tf2_ros
)


# ========================================
# 5. BUILD VISUALIZATION TOOL (Optional but useful)
# ========================================
add_executable(registration_visualisation
  src/registration_visualisation.cpp
  src/visualise.cpp
)
target_link_libraries(registration_visualisation map_merging ${PCL_LIBRARIES})
ament_target_dependencies(registration_visualisation
  rclcpp
  sensor_msgs
  pcl_conversions
)


# ========================================
# 6. INSTALL RULES
# ========================================
install(TARGETS
  map_merge_node
  map_merging
  registration_visualisation
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()