cmake_minimum_required(VERSION 3.5)
project(rosbag2_bag_v2_plugins)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Won't build on Windows
if(NOT UNIX)
  ament_package()
  return()
endif()

find_package(pluginlib REQUIRED)
find_package(rcl REQUIRED)
find_package(rcutils REQUIRED)
find_package(rosbag2 REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(ros1_rosbag_storage REQUIRED)  # provided by ros1_rosbag_storage_vendor
find_package(ros1_bridge REQUIRED)

set(generated_path "${CMAKE_BINARY_DIR}/generated")
set(generated_files "${generated_path}/convert_rosbag_message.cpp")

add_library(
  ${PROJECT_NAME} SHARED
  src/rosbag2_bag_v2_plugins/storage/rosbag_v2_storage.cpp
  ${generated_files})

ament_target_dependencies(${PROJECT_NAME}
  ros1_rosbag_storage
  rcl
  rosbag2_storage
  rosbag2
  ros1_bridge
  pluginlib
  ${ros2_message_packages}
  ${prefixed_ros1_message_packages})

target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE
  ROSBAG2_BAG_V2_PLUGINS_BUILDING_DLL)

pluginlib_export_plugin_description_file(rosbag2_storage plugin_description.xml)

install(
  DIRECTORY include/
  DESTINATION include)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rosbag2_storage)
ament_export_dependencies(rosbag2 ros1_rosbag_storage)

add_custom_command(
  OUTPUT ${generated_files}
  COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/src/generate_coverter_cpp.py
  --output-path "${generated_path}" --template-dir ${CMAKE_CURRENT_SOURCE_DIR}/src/rosbag2_bag_v2_plugins/storage
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

# find ROS 1 packages with messages
find_package(PkgConfig)
include(${ros1_bridge_DIR}/find_ros1_interface_packages.cmake)
include(${ros1_bridge_DIR}/find_ros1_package.cmake)
find_ros1_interface_packages(ros1_message_packages)

set(prefixed_ros1_message_packages "")
foreach(ros1_message_package ${ros1_message_packages})
  find_ros1_package(${ros1_message_package} REQUIRED)
  list(APPEND prefixed_ros1_message_packages "ros1_${ros1_message_package}")
endforeach()

# generate conversion methods
ament_index_get_resources(ros2_message_packages "rosidl_interfaces")
foreach(message_package ${ros2_message_packages})
  find_package(${message_package} REQUIRED)
  message(STATUS "Found ${message_package}: ${${message_package}_VERSION} (${${message_package}_DIR})")
endforeach()

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
