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

find_package(PkgConfig)
if(NOT PKG_CONFIG_FOUND)
  message(WARNING "Failed to find PkgConfig, skipping...")
  # call ament_package() to avoid ament_tools treating this as a plain CMake pkg
  ament_package()
  return()
endif()

# include bridge first to not create package if ros1 packages cannot be found
find_package(ros1_bridge REQUIRED)
include(${ros1_bridge_DIR}/find_ros1_interface_packages.cmake)
include(${ros1_bridge_DIR}/find_ros1_package.cmake)

find_ros1_package(roscpp)
if(NOT ros1_roscpp_FOUND)
  message(WARNING "Failed to find ROS 1 roscpp, skipping...")
  # call ament_package() to avoid ament_tools treating this as a plain CMake pkg
  ament_package()
  return()
endif()

find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(rosbag2 REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(ros1_rosbag_storage REQUIRED)  # provided by ros1_rosbag_storage_vendor

# Find transitive ros1 dependencies for ros1_rosbag_storage
find_ros1_package(cpp_common)
find_ros1_package(roscpp_serialization)
find_ros1_package(roscpp_traits)
find_ros1_package(rostime)
find_ros1_package(roslz4)

set(generated_path "${CMAKE_BINARY_DIR}/generated")
set(generated_files "${generated_path}/convert_rosbag_message.cpp")

add_library(
  ${PROJECT_NAME} SHARED
  src/rosbag2_bag_v2_plugins/converter/rosbag_v_2_deserializer_plugin.cpp
  src/rosbag2_bag_v2_plugins/storage/rosbag_output_stream.cpp
  src/rosbag2_bag_v2_plugins/storage/rosbag_v2_storage.cpp
  ${generated_files})

ament_target_dependencies(${PROJECT_NAME}
  ros1_rosbag_storage
  rclcpp
  rosbag2_storage
  rosbag2
  ros1_bridge
  pluginlib
  ros1_cpp_common
  ros1_roscpp
  ros1_roscpp_serialization
  ros1_roscpp_traits
  ros1_rostime
  ros1_roslz4
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
ament_export_dependencies(rosbag2 ros1_rosbag_storage rosbag2_storage)

add_custom_command(
  OUTPUT ${generated_files}
  COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/src/generate_coverter_cpp.py
  --output-path "${generated_path}" --template-dir ${CMAKE_CURRENT_SOURCE_DIR}/src/rosbag2_bag_v2_plugins/storage
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

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
  find_package(rosbag2_test_common REQUIRED)

  ament_lint_auto_find_test_dependencies()

  add_definitions(-D_SRC_RESOURCES_DIR_PATH="${CMAKE_CURRENT_SOURCE_DIR}/resources")

  ament_add_gmock(test_rosbag_v2_storage
    test/rosbag2_bag_v2_plugins/test_rosbag_v2_storage.cpp
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  if(TARGET test_rosbag_v2_storage)
    target_include_directories(test_rosbag_v2_storage
      PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
    target_link_libraries(test_rosbag_v2_storage ${PROJECT_NAME})
    ament_target_dependencies(test_rosbag_v2_storage
      rosbag2_test_common)
  endif()

  ament_add_gmock(test_rosbag_output_stream
    test/rosbag2_bag_v2_plugins/test_rosbag_output_stream.cpp
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  if(TARGET test_rosbag_output_stream)
    target_include_directories(test_rosbag_output_stream
      PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>)
    target_link_libraries(test_rosbag_output_stream ${PROJECT_NAME})
    ament_target_dependencies(test_rosbag_output_stream
      rosbag2_test_common)
  endif()
endif()

ament_package()
