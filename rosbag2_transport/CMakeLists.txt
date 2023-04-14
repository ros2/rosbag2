cmake_minimum_required(VERSION 3.5)
project(rosbag2_transport)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
if(CMAKE_BUILD_TYPE STREQUAL "Debug" AND MSVC)
  # /bigobj is needed to avoid error C1128:
  #   https://msdn.microsoft.com/en-us/library/8578y171.aspx
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
endif()

# Windows supplies macros for min and max by default. We should only use min and max from stl
if(WIN32)
  add_definitions(-DNOMINMAX)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(keyboard_handler REQUIRED)
find_package(rcl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(rmw REQUIRED)
find_package(rosbag2_compression REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_interfaces REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(shared_queues_vendor REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/rosbag2_transport/bag_rewrite.cpp
  src/rosbag2_transport/player.cpp
  src/rosbag2_transport/qos.cpp
  src/rosbag2_transport/reader_writer_factory.cpp
  src/rosbag2_transport/recorder.cpp
  src/rosbag2_transport/record_options.cpp
  src/rosbag2_transport/topic_filter.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(${PROJECT_NAME}
  keyboard_handler::keyboard_handler
  rcl::rcl
  rclcpp::rclcpp
  rcutils::rcutils
  rmw::rmw
  rosbag2_compression::rosbag2_compression
  rosbag2_cpp::rosbag2_cpp
  ${rosbag2_interfaces_TARGETS}
  rosbag2_storage::rosbag2_storage
  shared_queues_vendor::singleproducerconsumer
  shared_queues_vendor::concurrentqueue
  yaml-cpp
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "ROSBAG2_TRANSPORT_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export old-style CMake variables
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})

# Export modern CMake targets
ament_export_targets(export_${PROJECT_NAME})

ament_export_dependencies(
  keyboard_handler
  rosbag2_cpp
  rosbag2_compression
  rosbag2_interfaces
  shared_queues_vendor
  yaml_cpp_vendor)

function(create_tests_for_rmw_implementation)
  # disable the following tests for connext
  # due to slower discovery of nodes
  set(SKIP_TEST "")
  if(${rmw_implementation} MATCHES "rmw_connext(.*)")
    set(SKIP_TEST "SKIP_TEST")
  endif()

  rosbag2_transport_add_gmock(test_play
    test/rosbag2_transport/test_play.cpp
    LINK_LIBS rosbag2_transport
    AMENT_DEPS test_msgs rosbag2_test_common
    ${SKIP_TEST})

  rosbag2_transport_add_gmock(test_play_loop
    test/rosbag2_transport/test_play_loop.cpp
    LINK_LIBS rosbag2_transport
    AMENT_DEPS test_msgs rosbag2_test_common
    ${SKIP_TEST})

  rosbag2_transport_add_gmock(test_play_publish_clock
    test/rosbag2_transport/test_play_publish_clock.cpp
    LINK_LIBS rosbag2_transport
    AMENT_DEPS test_msgs rosbag2_test_common
    ${SKIP_TEST})

  rosbag2_transport_add_gmock(test_play_timing
    test/rosbag2_transport/test_play_timing.cpp
    LINK_LIBS rosbag2_transport
    AMENT_DEPS test_msgs rosbag2_test_common)

  rosbag2_transport_add_gmock(test_play_callbacks
    test/rosbag2_transport/test_play_callbacks.cpp
    LINK_LIBS rosbag2_transport
    AMENT_DEPS test_msgs rosbag2_test_common)

  rosbag2_transport_add_gmock(test_player_stop
    test/rosbag2_transport/test_player_stop.cpp
    LINK_LIBS rosbag2_transport
    AMENT_DEPS test_msgs rosbag2_test_common)

  rosbag2_transport_add_gmock(test_play_seek
    test/rosbag2_transport/test_play_seek.cpp
    LINK_LIBS rosbag2_transport
    AMENT_DEPS test_msgs rosbag2_test_common)

  rosbag2_transport_add_gmock(test_play_services
    test/rosbag2_transport/test_play_services.cpp
    LINK_LIBS rosbag2_transport
    AMENT_DEPS test_msgs rosbag2_test_common)

  rosbag2_transport_add_gmock(test_play_topic_remap
    test/rosbag2_transport/test_play_topic_remap.cpp
    LINK_LIBS rosbag2_transport
    AMENT_DEPS test_msgs rosbag2_test_common
    ${SKIP_TEST})

  rosbag2_transport_add_gmock(test_play_duration
      test/rosbag2_transport/test_play_duration.cpp
      INCLUDE_DIRS $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/rosbag2_transport>
      LINK_LIBS rosbag2_transport
      AMENT_DEPS test_msgs rosbag2_test_common)

  rosbag2_transport_add_gmock(test_play_next
      test/rosbag2_transport/test_play_next.cpp
      INCLUDE_DIRS $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/rosbag2_transport>
      LINK_LIBS rosbag2_transport
      AMENT_DEPS test_msgs rosbag2_test_common)

  rosbag2_transport_add_gmock(test_play_until
      test/rosbag2_transport/test_play_until.cpp
      INCLUDE_DIRS $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/rosbag2_transport>
      LINK_LIBS rosbag2_transport
      AMENT_DEPS test_msgs rosbag2_test_common)

  rosbag2_transport_add_gmock(test_burst
    test/rosbag2_transport/test_burst.cpp
    INCLUDE_DIRS $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/rosbag2_transport>
    LINK_LIBS rosbag2_transport
    AMENT_DEPS test_msgs rosbag2_test_common)

  rosbag2_transport_add_gmock(test_qos
    test/rosbag2_transport/test_qos.cpp
    INCLUDE_DIRS $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    LINK_LIBS rosbag2_transport
    AMENT_DEPS rosbag2_test_common yaml_cpp_vendor)

  rosbag2_transport_add_gmock(test_record
    test/rosbag2_transport/test_record.cpp
    INCLUDE_DIRS $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/rosbag2_transport>
    LINK_LIBS rosbag2_transport
    AMENT_DEPS test_msgs rosbag2_test_common
    ${SKIP_TEST})

  rosbag2_transport_add_gmock(test_record_all
    test/rosbag2_transport/test_record_all.cpp
    LINK_LIBS rosbag2_transport
    AMENT_DEPS test_msgs rosbag2_test_common)

  rosbag2_transport_add_gmock(test_record_all_ignore_leaf_topics
    test/rosbag2_transport/test_record_all_ignore_leaf_topics.cpp
    LINK_LIBS rosbag2_transport
    AMENT_DEPS test_msgs rosbag2_test_common)

  rosbag2_transport_add_gmock(test_record_all_include_unpublished_topics
    test/rosbag2_transport/test_record_all_include_unpublished_topics.cpp
    LINK_LIBS rosbag2_transport
    AMENT_DEPS test_msgs rosbag2_test_common)

  rosbag2_transport_add_gmock(test_record_all_no_discovery
    test/rosbag2_transport/test_record_all_no_discovery.cpp
    LINK_LIBS rosbag2_transport
    AMENT_DEPS test_msgs rosbag2_test_common)

  rosbag2_transport_add_gmock(test_record_all_use_sim_time
    test/rosbag2_transport/test_record_all_use_sim_time.cpp
    LINK_LIBS rosbag2_transport
    AMENT_DEPS test_msgs rosbag2_test_common)

  rosbag2_transport_add_gmock(test_keyboard_controls
    test/rosbag2_transport/test_keyboard_controls.cpp
    LINK_LIBS rosbag2_transport
    AMENT_DEPS test_msgs rosbag2_test_common)

  rosbag2_transport_add_gmock(test_record_regex
    test/rosbag2_transport/test_record_regex.cpp
    LINK_LIBS rosbag2_transport
    AMENT_DEPS test_msgs rosbag2_test_common
    ${SKIP_TEST})

  rosbag2_transport_add_gmock(test_record_services
    test/rosbag2_transport/test_record_services.cpp
    LINK_LIBS rosbag2_transport
    AMENT_DEPS test_msgs rosbag2_test_common)

  if(${rmw_implementation} MATCHES "rmw_cyclonedds(.*)")
    ament_add_test_label(test_play_services__rmw_cyclonedds_cpp xfail)
  endif()

  if(${rmw_implementation} MATCHES "rmw_fastrtps(.*)")
    ament_add_test_label(test_play_services__rmw_fastrtps_cpp xfail)
  endif()
endfunction()

if(BUILD_TESTING)
  add_definitions(-D_SRC_RESOURCES_DIR_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test/resources")

  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(test_msgs REQUIRED)
  find_package(rosbag2_test_common REQUIRED)
  include(cmake/rosbag2_transport_add_gmock.cmake)
  ament_lint_auto_find_test_dependencies()
  call_for_each_rmw_implementation(create_tests_for_rmw_implementation)

  ament_add_gmock(test_record_options
    test/rosbag2_transport/test_record_options.cpp)
  target_link_libraries(test_record_options ${PROJECT_NAME})

  ament_add_gmock(test_type_description_hash
    test/rosbag2_transport/test_type_description_hash.cpp)
  target_link_libraries(test_type_description_hash ${PROJECT_NAME})

  ament_add_gmock(test_topic_filter
    test/rosbag2_transport/test_topic_filter.cpp)
  target_include_directories(test_topic_filter PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/rosbag2_transport>)
  target_link_libraries(test_topic_filter
    ${PROJECT_NAME}
  )

  ament_add_gmock(test_rewrite
    test/rosbag2_transport/test_rewrite.cpp)
  target_link_libraries(test_rewrite
    ${PROJECT_NAME}
    keyboard_handler::keyboard_handler
    rcpputils::rcpputils
    rosbag2_cpp::rosbag2_cpp
    rosbag2_test_common::rosbag2_test_common
    ${test_msgs_TARGTES}
  )
endif()

ament_package()
