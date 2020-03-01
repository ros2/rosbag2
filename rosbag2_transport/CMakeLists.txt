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
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# Windows supplies macros for min and max by default. We should only use min and max from stl
if(WIN32)
  add_definitions(-DNOMINMAX)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rcl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(rmw REQUIRED)
find_package(rosbag2_compression REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(shared_queues_vendor REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/rosbag2_transport/converter.cpp
  src/rosbag2_transport/player.cpp
  src/rosbag2_transport/formatter.cpp
  src/rosbag2_transport/generic_publisher.cpp
  src/rosbag2_transport/generic_subscription.cpp
  src/rosbag2_transport/recorder.cpp
  src/rosbag2_transport/rosbag2_node.cpp
  src/rosbag2_transport/rosbag2_transport.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}
  rcl
  rclcpp
  rcutils
  rmw
  rosbag2_compression
  rosbag2_cpp
  rosidl_generator_cpp
  shared_queues_vendor
)

include(cmake/configure_python.cmake)
ament_python_install_package(${PROJECT_NAME})
add_library(rosbag2_transport_py
  SHARED
  src/rosbag2_transport/rosbag2_transport_python.cpp)
configure_python_c_extension_library(rosbag2_transport_py)
target_link_libraries(rosbag2_transport_py rosbag2_transport)
ament_target_dependencies(rosbag2_transport_py rosbag2_compression)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "ROSBAG2_TRANSPORT_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_interfaces(export_${PROJECT_NAME})
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rosbag2_cpp rosbag2_compression)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(test_msgs REQUIRED)
  find_package(rosbag2_test_common REQUIRED)
  # explicitly disable cppcheck
  set(ament_cmake_cppcheck_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gmock(test_info
    test/rosbag2_transport/test_info.cpp
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  if(TARGET test_info)
    target_link_libraries(test_info rosbag2_transport)
    ament_target_dependencies(test_info rosbag2_test_common)
  endif()

  ament_add_gmock(test_record_all
    test/rosbag2_transport/test_record_all.cpp
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  if(TARGET test_record_all)
    target_link_libraries(test_record_all rosbag2_transport)
    ament_target_dependencies(test_record_all test_msgs rosbag2_test_common)
  endif()

  ament_add_gmock(test_record_all_no_discovery
    test/rosbag2_transport/test_record_all_no_discovery.cpp
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  if(TARGET test_record_all_no_discovery)
    target_link_libraries(test_record_all_no_discovery rosbag2_transport)
    ament_target_dependencies(test_record_all_no_discovery test_msgs rosbag2_test_common)
  endif()

  ament_add_gmock(test_play_timing
    test/rosbag2_transport/test_play_timing.cpp
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  if(TARGET test_play_timing)
    target_link_libraries(test_play_timing rosbag2_transport)
    ament_target_dependencies(test_play_timing test_msgs rosbag2_test_common)
  endif()

  ament_add_gmock(test_rosbag2_node
    test/rosbag2_transport/test_rosbag2_node.cpp
    src/rosbag2_transport/generic_publisher.cpp
    src/rosbag2_transport/generic_subscription.cpp
    src/rosbag2_transport/rosbag2_node.cpp
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  if(TARGET test_rosbag2_node)
    target_include_directories(test_rosbag2_node
      PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>)
    ament_target_dependencies(test_rosbag2_node
      ament_index_cpp
      rosbag2_cpp
      rclcpp
      test_msgs
      rosbag2_test_common)
  endif()

  ament_add_gmock(test_formatter
    test/rosbag2_transport/test_formatter.cpp
    src/rosbag2_transport/formatter.cpp
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  if(TARGET test_formatter)
    target_link_libraries(test_formatter rosbag2_transport)
  endif()

  # disable the following tests for connext
  # due to slower discovery of nodes
  get_default_rmw_implementation(rmw_default)
  set(SKIP_TEST "")
  if(${rmw_default} MATCHES "(.*)connext(.*)")
    set(SKIP_TEST "SKIP_TEST")
  endif()

  ament_add_gmock(test_record
    test/rosbag2_transport/test_record.cpp
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    ${SKIP_TEST})
  if(TARGET test_record)
    target_link_libraries(test_record rosbag2_transport)
    ament_target_dependencies(test_record test_msgs rosbag2_test_common)
  endif()

  ament_add_gmock(test_play
    test/rosbag2_transport/test_play.cpp
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    ${SKIP_TEST})
  if(TARGET test_play)
    target_link_libraries(test_play rosbag2_transport)
    ament_target_dependencies(test_play test_msgs rosbag2_test_common)
  endif()

endif()

ament_package()
