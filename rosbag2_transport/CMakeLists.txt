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
find_package(rosbag2_storage REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(shared_queues_vendor REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/rosbag2_transport/player.cpp
  src/rosbag2_transport/generic_publisher.cpp
  src/rosbag2_transport/generic_subscription.cpp
  src/rosbag2_transport/qos.cpp
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
  rosbag2_storage
  shared_queues_vendor
  yaml_cpp_vendor
)

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
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(rosbag2_cpp rosbag2_compression shared_queues_vendor yaml_cpp_vendor)

function(create_tests_for_rmw_implementation)
  rosbag2_transport_add_gmock(test_record_all
    test/rosbag2_transport/test_record_all.cpp
    LINK_LIBS rosbag2_transport
    AMENT_DEPS rosbag2_cpp rosbag2_storage test_msgs rosbag2_test_common)

  rosbag2_transport_add_gmock(test_record_all_no_discovery
    test/rosbag2_transport/test_record_all_no_discovery.cpp
    LINK_LIBS rosbag2_transport
    AMENT_DEPS rosbag2_cpp rosbag2_storage test_msgs rosbag2_test_common)

  rosbag2_transport_add_gmock(test_play_timing
    test/rosbag2_transport/test_play_timing.cpp
    LINK_LIBS rosbag2_transport
    AMENT_DEPS rosbag2_cpp rosbag2_storage test_msgs rosbag2_test_common)

  rosbag2_transport_add_gmock(test_rosbag2_node
    src/rosbag2_transport/generic_publisher.cpp
    src/rosbag2_transport/generic_subscription.cpp
    src/rosbag2_transport/qos.cpp
    src/rosbag2_transport/recorder.cpp
    src/rosbag2_transport/rosbag2_node.cpp
    test/rosbag2_transport/test_rosbag2_node.cpp
    INCLUDE_DIRS
      PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/rosbag2_transport>
      $<INSTALL_INTERFACE:include>
    AMENT_DEPS
      ament_index_cpp
      rclcpp
      rosbag2_cpp
      rosbag2_storage
      rosbag2_test_common
      test_msgs
      yaml_cpp_vendor)

  rosbag2_transport_add_gmock(test_qos
    src/rosbag2_transport/qos.cpp
    test/rosbag2_transport/test_qos.cpp
    INCLUDE_DIRS
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/rosbag2_transport>
    AMENT_DEPS
      rclcpp
      rosbag2_test_common
      yaml_cpp_vendor)

  # disable the following tests for connext
  # due to slower discovery of nodes
  set(SKIP_TEST "")
  if(${rmw_implementation} MATCHES "rmw_connext(.*)_cpp")
    set(SKIP_TEST "SKIP_TEST")
  endif()

  rosbag2_transport_add_gmock(test_record
    test/rosbag2_transport/test_record.cpp
    AMENT_DEPS rosbag2_cpp rosbag2_storage test_msgs rosbag2_test_common
    INCLUDE_DIRS $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/rosbag2_transport>
    LINK_LIBS rosbag2_transport
    ${SKIP_TEST})

  rosbag2_transport_add_gmock(test_record_regex
    test/rosbag2_transport/test_record_regex.cpp
    LINK_LIBS rosbag2_transport
    AMENT_DEPS test_msgs rosbag2_test_common
    ${SKIP_TEST})

  rosbag2_transport_add_gmock(test_play
    src/rosbag2_transport/qos.cpp
    test/rosbag2_transport/test_play.cpp
    INCLUDE_DIRS $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/rosbag2_transport>
    LINK_LIBS rosbag2_transport
    AMENT_DEPS rosbag2_cpp rosbag2_storage test_msgs rosbag2_test_common
    ${SKIP_TEST})

  rosbag2_transport_add_gmock(test_play_loop
    test/rosbag2_transport/test_play_loop.cpp
    ${SKIP_TEST}
    LINK_LIBS rosbag2_transport
    AMENT_DEPS rosbag2_cpp rosbag2_storage test_msgs rosbag2_test_common)

  rosbag2_transport_add_gmock(test_play_topic_remap
    test/rosbag2_transport/test_play_topic_remap.cpp
    ${SKIP_TEST}
    LINK_LIBS rosbag2_transport
    AMENT_DEPS rosbag2_cpp rosbag2_storage test_msgs rosbag2_test_common)
endfunction()

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(test_msgs REQUIRED)
  find_package(rosbag2_test_common REQUIRED)
  include(cmake/rosbag2_transport_add_gmock.cmake)
  ament_lint_auto_find_test_dependencies()
  call_for_each_rmw_implementation(create_tests_for_rmw_implementation)
endif()

ament_package()
