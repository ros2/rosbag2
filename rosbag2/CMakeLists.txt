cmake_minimum_required(VERSION 3.20)
project(rosbag2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosbag2_transport REQUIRED)

# Library for Rosbag2 composable nodes
add_library(${PROJECT_NAME}_nodes SHARED
  src/rosbag2/composable_player.cpp
  src/rosbag2/composable_recorder.cpp
)
target_include_directories(${PROJECT_NAME}_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

ament_target_dependencies(${PROJECT_NAME}_nodes rclcpp rclcpp_components rosbag2_transport)

rclcpp_components_register_node(${PROJECT_NAME}_nodes PLUGIN "rosbag2::Player" EXECUTABLE player)

rclcpp_components_register_node(${PROJECT_NAME}_nodes PLUGIN "rosbag2::Recorder" EXECUTABLE recorder)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}_nodes
  EXPORT export_${PROJECT_NAME}_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export modern CMake targets
ament_export_targets(export_${PROJECT_NAME}_nodes)

ament_package()
