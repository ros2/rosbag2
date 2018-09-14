cmake_minimum_required(VERSION 3.5)
project(rosbag2_transport)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rcl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(rosbag2 REQUIRED)
find_package(rosbag2_storage REQUIRED)  # TODO(MI) delete
find_package(shared_queues_vendor REQUIRED)

add_library(rosbag2_transport
  src/rosbag2_transport/player.cpp
  src/rosbag2_transport/generic_publisher.cpp
  src/rosbag2_transport/generic_subscription.cpp
  src/rosbag2_transport/rosbag2_node.cpp
  src/rosbag2_transport/rosbag2_transport.cpp)
target_include_directories(rosbag2_transport PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(rosbag2_transport
  rcl
  rclcpp
  rcutils
  rosbag2
  rosbag2_storage
  rosidl_generator_c
  shared_queues_vendor
)

include(cmake/configure_python.cmake)
ament_python_install_package(${PROJECT_NAME})
add_library(rosbag2_transport_py
  SHARED
  src/rosbag2_transport/rosbag2_transport_python.cpp)
configure_python_c_extension_library(rosbag2_transport_py)
target_link_libraries(rosbag2_transport_py rosbag2_transport)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(rosbag2_transport PRIVATE "ROSBAG2_TRANSPORT_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS rosbag2_transport
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_interfaces(export_${PROJECT_NAME})
ament_export_libraries(rosbag2_transport)
ament_export_dependencies(rosbag2)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(rosbag2_storage_default_plugins REQUIRED)
  find_package(test_msgs REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gmock(rosbag2_record_integration_test
    test/rosbag2_transport/rosbag2_record_integration_test.cpp
    test/rosbag2_transport/test_memory_management.cpp
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  if(TARGET rosbag2_record_integration_test)
    target_link_libraries(rosbag2_record_integration_test rosbag2_transport)
    ament_target_dependencies(rosbag2_record_integration_test test_msgs
      rosbag2_storage_default_plugins)
  endif()

  ament_add_gmock(rosbag2_record_all_integration_test
    test/rosbag2_transport/rosbag2_record_all_integration_test.cpp
    test/rosbag2_transport/test_memory_management.cpp
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  if(TARGET rosbag2_record_all_integration_test)
    target_link_libraries(rosbag2_record_all_integration_test rosbag2_transport)
    ament_target_dependencies(rosbag2_record_all_integration_test test_msgs
      rosbag2_storage_default_plugins)
  endif()

  ament_add_gmock(rosbag2_play_integration_test
    test/rosbag2_transport/rosbag2_play_integration_test.cpp
    test/rosbag2_transport/test_memory_management.cpp
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  if(TARGET rosbag2_play_integration_test)
    target_link_libraries(rosbag2_play_integration_test rosbag2_transport)
    ament_target_dependencies(rosbag2_play_integration_test test_msgs)
  endif()

  ament_add_gmock(rosbag2_play_timing_integration_test
    test/rosbag2_transport/rosbag2_play_timing_integration_test.cpp
    test/rosbag2_transport/test_memory_management.cpp
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  if(TARGET rosbag2_play_timing_integration_test)
    target_link_libraries(rosbag2_play_timing_integration_test rosbag2_transport)
    ament_target_dependencies(rosbag2_play_timing_integration_test test_msgs)
  endif()

  ament_add_gmock(rosbag2_rosbag_node_test
    test/rosbag2_transport/rosbag2_rosbag_node_test.cpp
    test/rosbag2_transport/test_memory_management.cpp
    src/rosbag2_transport/generic_publisher.cpp
    src/rosbag2_transport/generic_subscription.cpp
    src/rosbag2_transport/rosbag2_node.cpp
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  if(TARGET rosbag2_rosbag_node_test)
    target_include_directories(rosbag2_rosbag_node_test
      PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>)
    ament_target_dependencies(rosbag2_rosbag_node_test
      ament_index_cpp
      rosbag2
      rclcpp
      test_msgs)
  endif()
endif()

ament_package()
