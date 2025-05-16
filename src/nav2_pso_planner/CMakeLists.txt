cmake_minimum_required(VERSION 3.5)
project(nav2_pso_planner)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

nav2_package() #Calls the nav2_package.cmake file


include_directories(
  include
)

set(library_name nav2_pso_planner)

set(dependencies ament_cmake
  builtin_interfaces
  nav2_common
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav_msgs
  geometry_msgs
  nav2_costmap_2d
  pluginlib
  tf2_ros
  tf2_geometry_msgs
  nav2_util
)

add_library(${library_name} SHARED
  src/pso_planner.cpp
  src/pso.cpp
  src/TrajectoryGeneration.cpp
)

ament_target_dependencies(${library_name} ${dependencies})

pluginlib_export_plugin_description_file(nav2_core pso_planner.xml)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES pso_planner.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY params/
  DESTINATION share/${PROJECT_NAME}/params
)


ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()
