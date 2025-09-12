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

find_package(ament_cmake_ros REQUIRED)
find_package(keyboard_handler REQUIRED)
find_package(rcl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcpputils REQUIRED)
find_package(rcutils REQUIRED)
find_package(rmw REQUIRED)
find_package(rosbag2_compression REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_interfaces REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/rosbag2_transport/bag_rewrite.cpp
  src/rosbag2_transport/recorder_event_notifier.cpp
  src/rosbag2_transport/player.cpp
  src/rosbag2_transport/play_options.cpp
  src/rosbag2_transport/player_service_client.cpp
  src/rosbag2_transport/reader_writer_factory.cpp
  src/rosbag2_transport/readers_manager.cpp
  src/rosbag2_transport/recorder.cpp
  src/rosbag2_transport/record_options.cpp
  src/rosbag2_transport/topic_filter.cpp
  src/rosbag2_transport/config_options_from_node_params.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(${PROJECT_NAME} PUBLIC
  keyboard_handler::keyboard_handler
  rclcpp::rclcpp
  rosbag2_cpp::rosbag2_cpp
  ${rosbag2_interfaces_TARGETS}
  rosbag2_storage::rosbag2_storage
)
target_link_libraries(${PROJECT_NAME} PRIVATE
  rcl::rcl
  rclcpp_components::component
  rcpputils::rcpputils
  rcutils::rcutils
  rmw::rmw
  rosbag2_compression::rosbag2_compression
  yaml-cpp::yaml-cpp
)

rclcpp_components_register_node(
  ${PROJECT_NAME} PLUGIN "rosbag2_transport::Player" EXECUTABLE player)

rclcpp_components_register_node(
  ${PROJECT_NAME} PLUGIN "rosbag2_transport::Recorder" EXECUTABLE recorder)

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
  rclcpp
  rosbag2_cpp
  rosbag2_interfaces
  rosbag2_storage
)

if(BUILD_TESTING)
  add_definitions(-D_SRC_RESOURCES_DIR_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test/resources")

  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(composition_interfaces REQUIRED)
  find_package(rosbag2_test_common REQUIRED)
  find_package(test_msgs REQUIRED)

  ament_lint_auto_find_test_dependencies()

  # finding gtest once in the highest scope
  # prevents finding it repeatedly in each local scope
  ament_find_gmock()

  ament_add_gmock_executable(test_play test/rosbag2_transport/test_play.cpp)
  target_link_libraries(test_play
    rosbag2_transport
    ${test_msgs_TARGETS}
    rosbag2_test_common::rosbag2_test_common
    rcutils::rcutils
  )
  function(test_play_for_rmw_implementation)
    set(rmw_implementation_env_var RMW_IMPLEMENTATION=${rmw_implementation})

    # disable the following tests for connext
    # due to slower discovery of nodes
    set(SKIP_TEST "")
    if(${rmw_implementation} MATCHES "rmw_connext(.*)")
      set(SKIP_TEST "SKIP_TEST")
    endif()

    ament_add_gmock_test(test_play
      TEST_NAME test_play${target_suffix}
      ENV ${rmw_implementation_env_var}
      ${SKIP_TEST}
    )
  endfunction()
  call_for_each_rmw_implementation(test_play_for_rmw_implementation)

  ament_add_gmock_executable(test_composable_player test/rosbag2_transport/test_composable_player.cpp)
  target_link_libraries(test_composable_player
    ${composition_interfaces_TARGETS}
    rclcpp_components::component_manager
    rosbag2_transport
    rosbag2_test_common::rosbag2_test_common
    ${test_msgs_TARGETS}
  )
  function(test_composable_player_for_rmw_implementation)
    set(rmw_implementation_env_var RMW_IMPLEMENTATION=${rmw_implementation})

    ament_add_gmock_test(test_composable_player
      TEST_NAME test_composable_player${target_suffix}
      ENV ${rmw_implementation_env_var}
    )
  endfunction()
  call_for_each_rmw_implementation(test_composable_player_for_rmw_implementation)

  ament_add_gmock_executable(test_play_loop test/rosbag2_transport/test_play_loop.cpp)
  target_link_libraries(test_play_loop rosbag2_transport ${test_msgs_TARGETS} rosbag2_test_common::rosbag2_test_common)
  function(test_play_loop_for_rmw_implementation)
    set(rmw_implementation_env_var RMW_IMPLEMENTATION=${rmw_implementation})

    # disable the following tests for connext
    # due to slower discovery of nodes
    set(SKIP_TEST "")
    if(${rmw_implementation} MATCHES "rmw_connext(.*)")
      set(SKIP_TEST "SKIP_TEST")
    endif()

    ament_add_gmock_test(test_play_loop
      TEST_NAME test_play_loop${target_suffix}
      ENV ${rmw_implementation_env_var}
      ${SKIP_TEST}
    )
  endfunction()
  call_for_each_rmw_implementation(test_play_loop_for_rmw_implementation)

  ament_add_gmock_executable(test_play_publish_clock test/rosbag2_transport/test_play_publish_clock.cpp)
  target_link_libraries(test_play_publish_clock rosbag2_transport ${test_msgs_TARGETS} rosbag2_test_common::rosbag2_test_common)
  function(test_play_publish_clock_for_rmw_implementation)
    set(rmw_implementation_env_var RMW_IMPLEMENTATION=${rmw_implementation})

    # disable the following tests for connext
    # due to slower discovery of nodes
    set(SKIP_TEST "")
    if(${rmw_implementation} MATCHES "rmw_connext(.*)")
      set(SKIP_TEST "SKIP_TEST")
    endif()

    ament_add_gmock_test(test_play_publish_clock
      TEST_NAME test_play_publish_clock${target_suffix}
      ENV ${rmw_implementation_env_var}
      ${SKIP_TEST}
    )
  endfunction()
  call_for_each_rmw_implementation(test_play_publish_clock_for_rmw_implementation)

  ament_add_gmock_executable(test_play_timing test/rosbag2_transport/test_play_timing.cpp)
  target_link_libraries(test_play_timing rosbag2_transport ${test_msgs_TARGETS} rosbag2_test_common::rosbag2_test_common)
  function(test_play_timing_for_rmw_implementation)
    set(rmw_implementation_env_var RMW_IMPLEMENTATION=${rmw_implementation})

    ament_add_gmock_test(test_play_timing
      TEST_NAME test_play_timing${target_suffix}
      ENV ${rmw_implementation_env_var}
    )
  endfunction()
  call_for_each_rmw_implementation(test_play_timing_for_rmw_implementation)

  ament_add_gmock_executable(test_play_callbacks test/rosbag2_transport/test_play_callbacks.cpp)
  target_link_libraries(test_play_callbacks rosbag2_transport ${test_msgs_TARGETS} rosbag2_test_common::rosbag2_test_common)
  function(test_play_callbacks_for_rmw_implementation)
    set(rmw_implementation_env_var RMW_IMPLEMENTATION=${rmw_implementation})

    ament_add_gmock_test(test_play_callbacks
      TEST_NAME test_play_callbacks${target_suffix}
      ENV ${rmw_implementation_env_var}
    )
  endfunction()
  call_for_each_rmw_implementation(test_play_callbacks_for_rmw_implementation)

  ament_add_gmock_executable(test_player_stop test/rosbag2_transport/test_player_stop.cpp)
  target_link_libraries(test_player_stop rosbag2_transport ${test_msgs_TARGETS} rosbag2_test_common::rosbag2_test_common rmw::rmw)
  function(test_player_stop_for_rmw_implementation)
    set(rmw_implementation_env_var RMW_IMPLEMENTATION=${rmw_implementation})

    ament_add_gmock_test(test_player_stop
      TEST_NAME test_player_stop${target_suffix}
      ENV ${rmw_implementation_env_var}
    )
  endfunction()
  call_for_each_rmw_implementation(test_player_stop_for_rmw_implementation)

  ament_add_gmock_executable(test_play_seek test/rosbag2_transport/test_play_seek.cpp)
  target_link_libraries(test_play_seek rosbag2_transport ${test_msgs_TARGETS} rosbag2_test_common::rosbag2_test_common)
  function(test_play_seek_for_rmw_implementation)
    set(rmw_implementation_env_var RMW_IMPLEMENTATION=${rmw_implementation})

    ament_add_gmock_test(test_play_seek
      TEST_NAME test_play_seek${target_suffix}
      ENV ${rmw_implementation_env_var}
    )
  endfunction()
  call_for_each_rmw_implementation(test_play_seek_for_rmw_implementation)

  ament_add_gmock_executable(test_play_services test/rosbag2_transport/test_play_services.cpp)
  target_link_libraries(test_play_services rosbag2_transport ${test_msgs_TARGETS} rosbag2_test_common::rosbag2_test_common)
  function(test_play_services_for_rmw_implementation)
    set(rmw_implementation_env_var RMW_IMPLEMENTATION=${rmw_implementation})

    ament_add_gmock_test(test_play_services
      TEST_NAME test_play_services${target_suffix}
      ENV ${rmw_implementation_env_var}
    )
  endfunction()
  call_for_each_rmw_implementation(test_play_services_for_rmw_implementation)

  ament_add_gmock_executable(test_play_topic_remap test/rosbag2_transport/test_play_topic_remap.cpp)
  target_link_libraries(test_play_topic_remap rosbag2_transport ${test_msgs_TARGETS} rosbag2_test_common::rosbag2_test_common)
  function(test_play_topic_remap_for_rmw_implementation)
    set(rmw_implementation_env_var RMW_IMPLEMENTATION=${rmw_implementation})

    # disable the following tests for connext
    # due to slower discovery of nodes
    set(SKIP_TEST "")
    if(${rmw_implementation} MATCHES "rmw_connext(.*)")
      set(SKIP_TEST "SKIP_TEST")
    endif()

    ament_add_gmock_test(test_play_topic_remap
      TEST_NAME test_play_topic_remap${target_suffix}
      ENV ${rmw_implementation_env_var}
      ${SKIP_TEST}
    )
  endfunction()
  call_for_each_rmw_implementation(test_play_topic_remap_for_rmw_implementation)

  ament_add_gmock_executable(test_play_duration test/rosbag2_transport/test_play_duration.cpp)
  target_include_directories(test_play_duration PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/rosbag2_transport>)
  target_link_libraries(test_play_duration rosbag2_transport ${test_msgs_TARGETS} rosbag2_test_common::rosbag2_test_common)
  function(test_play_duration_for_rmw_implementation)
    set(rmw_implementation_env_var RMW_IMPLEMENTATION=${rmw_implementation})

    ament_add_gmock_test(test_play_duration
      TEST_NAME test_play_duration${target_suffix}
      ENV ${rmw_implementation_env_var}
    )
  endfunction()
  call_for_each_rmw_implementation(test_play_duration_for_rmw_implementation)

  ament_add_gmock_executable(test_play_next test/rosbag2_transport/test_play_next.cpp)
  target_include_directories(test_play_next PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/rosbag2_transport>)
  target_link_libraries(test_play_next rosbag2_transport ${test_msgs_TARGETS} rosbag2_test_common::rosbag2_test_common)
  function(test_play_next_for_rmw_implementation)
    set(rmw_implementation_env_var RMW_IMPLEMENTATION=${rmw_implementation})

    ament_add_gmock_test(test_play_next
      TEST_NAME test_play_next${target_suffix}
      ENV ${rmw_implementation_env_var}
    )
  endfunction()
  call_for_each_rmw_implementation(test_play_next_for_rmw_implementation)

  ament_add_gmock_executable(test_play_until test/rosbag2_transport/test_play_until.cpp)
  target_include_directories(test_play_until PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/rosbag2_transport>)
  target_link_libraries(test_play_until rosbag2_transport ${test_msgs_TARGETS} rosbag2_test_common::rosbag2_test_common)
  function(test_play_until_for_rmw_implementation)
    set(rmw_implementation_env_var RMW_IMPLEMENTATION=${rmw_implementation})

    ament_add_gmock_test(test_play_until
      TEST_NAME test_play_until${target_suffix}
      ENV ${rmw_implementation_env_var}
    )
  endfunction()
  call_for_each_rmw_implementation(test_play_until_for_rmw_implementation)

  ament_add_gmock_executable(test_burst test/rosbag2_transport/test_burst.cpp)
  target_include_directories(test_burst PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/rosbag2_transport>)
  target_link_libraries(test_burst rosbag2_transport ${test_msgs_TARGETS} rosbag2_test_common::rosbag2_test_common)
  function(test_burst_for_rmw_implementation)
    set(rmw_implementation_env_var RMW_IMPLEMENTATION=${rmw_implementation})

    ament_add_gmock_test(test_burst
      TEST_NAME test_burst${target_suffix}
      ENV ${rmw_implementation_env_var}
    )
  endfunction()
  call_for_each_rmw_implementation(test_burst_for_rmw_implementation)

  ament_add_gmock_executable(test_record test/rosbag2_transport/test_record.cpp)
  target_include_directories(test_record PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/rosbag2_transport>)
  target_link_libraries(test_record rosbag2_transport ${test_msgs_TARGETS} rosbag2_test_common::rosbag2_test_common rmw::rmw)
  function(test_record_for_rmw_implementation)
    set(rmw_implementation_env_var RMW_IMPLEMENTATION=${rmw_implementation})

    # disable the following tests for connext
    # due to slower discovery of nodes
    set(SKIP_TEST "")
    if(${rmw_implementation} MATCHES "rmw_connext(.*)")
      set(SKIP_TEST "SKIP_TEST")
    endif()

    ament_add_gmock_test(test_record
      TEST_NAME test_record${target_suffix}
      ENV ${rmw_implementation_env_var}
      ${SKIP_TEST}
    )
  endfunction()
  call_for_each_rmw_implementation(test_record_for_rmw_implementation)

  ament_add_gmock_executable(test_record_all test/rosbag2_transport/test_record_all.cpp)
  target_link_libraries(test_record_all rosbag2_transport ${test_msgs_TARGETS} rosbag2_test_common::rosbag2_test_common)
  function(test_record_all_for_rmw_implementation)
    set(rmw_implementation_env_var RMW_IMPLEMENTATION=${rmw_implementation})

    ament_add_gmock_test(test_record_all
      TEST_NAME test_record_all${target_suffix}
      ENV ${rmw_implementation_env_var}
    )
  endfunction()
  call_for_each_rmw_implementation(test_record_all_for_rmw_implementation)

  ament_add_gmock_executable(test_record_all_ignore_leaf_topics test/rosbag2_transport/test_record_all_ignore_leaf_topics.cpp)
  target_link_libraries(test_record_all_ignore_leaf_topics rosbag2_transport ${test_msgs_TARGETS} rosbag2_test_common::rosbag2_test_common)
  function(test_record_all_ignore_leaf_topics_for_rmw_implementation)
    set(rmw_implementation_env_var RMW_IMPLEMENTATION=${rmw_implementation})

    ament_add_gmock_test(test_record_all_ignore_leaf_topics
      TEST_NAME test_record_all_ignore_leaf_topics${target_suffix}
      ENV ${rmw_implementation_env_var}
    )
  endfunction()
  call_for_each_rmw_implementation(test_record_all_ignore_leaf_topics_for_rmw_implementation)

  ament_add_gmock_executable(test_record_all_include_unpublished_topics
    test/rosbag2_transport/test_record_all_include_unpublished_topics.cpp)
  target_link_libraries(test_record_all_include_unpublished_topics
    rosbag2_transport
    ${test_msgs_TARGETS}
    rosbag2_test_common::rosbag2_test_common
  )
  function(test_record_all_include_unpublished_topics_for_rmw_implementation)
    set(rmw_implementation_env_var RMW_IMPLEMENTATION=${rmw_implementation})

    ament_add_gmock_test(test_record_all_include_unpublished_topics
      TEST_NAME test_record_all_include_unpublished_topics${target_suffix}
      ENV ${rmw_implementation_env_var}
    )
  endfunction()
  call_for_each_rmw_implementation(test_record_all_include_unpublished_topics_for_rmw_implementation)

  ament_add_gmock_executable(test_record_all_no_discovery test/rosbag2_transport/test_record_all_no_discovery.cpp)
  target_link_libraries(test_record_all_no_discovery rosbag2_transport ${test_msgs_TARGETS} rosbag2_test_common::rosbag2_test_common)
  function(test_record_all_no_discovery_for_rmw_implementation)
    set(rmw_implementation_env_var RMW_IMPLEMENTATION=${rmw_implementation})

    ament_add_gmock_test(test_record_all_no_discovery
      TEST_NAME test_record_all_no_discovery${target_suffix}
      ENV ${rmw_implementation_env_var}
    )
  endfunction()
  call_for_each_rmw_implementation(test_record_all_no_discovery_for_rmw_implementation)

  ament_add_gmock_executable(test_record_all_use_sim_time test/rosbag2_transport/test_record_all_use_sim_time.cpp)
  target_link_libraries(test_record_all_use_sim_time rosbag2_transport ${test_msgs_TARGETS} rosbag2_test_common::rosbag2_test_common)
  function(test_record_all_use_sim_time_for_rmw_implementation)
    set(rmw_implementation_env_var RMW_IMPLEMENTATION=${rmw_implementation})

    ament_add_gmock_test(test_record_all_use_sim_time
      TEST_NAME test_record_all_use_sim_time${target_suffix}
      ENV ${rmw_implementation_env_var}
    )
  endfunction()
  call_for_each_rmw_implementation(test_record_all_use_sim_time_for_rmw_implementation)

  ament_add_gmock_executable(test_keyboard_controls test/rosbag2_transport/test_keyboard_controls.cpp)
  target_link_libraries(test_keyboard_controls rosbag2_transport ${test_msgs_TARGETS} rosbag2_test_common::rosbag2_test_common)
  function(test_keyboard_controls_for_rmw_implementation)
    set(rmw_implementation_env_var RMW_IMPLEMENTATION=${rmw_implementation})

    ament_add_gmock_test(test_keyboard_controls
      TEST_NAME test_keyboard_controls${target_suffix}
      ENV ${rmw_implementation_env_var}
    )
  endfunction()
  call_for_each_rmw_implementation(test_keyboard_controls_for_rmw_implementation)

  ament_add_gmock_executable(test_record_regex test/rosbag2_transport/test_record_regex.cpp)
  target_link_libraries(test_record_regex rosbag2_transport ${test_msgs_TARGETS} rosbag2_test_common::rosbag2_test_common)
  function(test_record_regex_for_rmw_implementation)
    set(rmw_implementation_env_var RMW_IMPLEMENTATION=${rmw_implementation})

    # disable the following tests for connext
    # due to slower discovery of nodes
    set(SKIP_TEST "")
    if(${rmw_implementation} MATCHES "rmw_connext(.*)")
      set(SKIP_TEST "SKIP_TEST")
    endif()

    ament_add_gmock_test(test_record_regex
      TEST_NAME test_record_regex${target_suffix}
      ENV ${rmw_implementation_env_var}
      ${SKIP_TEST}
    )
  endfunction()
  call_for_each_rmw_implementation(test_record_regex_for_rmw_implementation)

  ament_add_gmock_executable(test_record_services test/rosbag2_transport/test_record_services.cpp)
  target_link_libraries(test_record_services rosbag2_transport ${test_msgs_TARGETS} rosbag2_test_common::rosbag2_test_common)
  function(test_record_services_for_rmw_implementation)
    set(rmw_implementation_env_var RMW_IMPLEMENTATION=${rmw_implementation})

    ament_add_gmock_test(test_record_services
      TEST_NAME test_record_services${target_suffix}
      ENV ${rmw_implementation_env_var}
    )
  endfunction()
  call_for_each_rmw_implementation(test_record_services_for_rmw_implementation)

  ament_add_gmock_executable(test_composable_recorder test/rosbag2_transport/test_composable_recorder.cpp)
  target_link_libraries(test_composable_recorder
    ${composition_interfaces_TARGETS}
    rclcpp_components::component_manager
    rosbag2_test_common::rosbag2_test_common
    rosbag2_transport
    ${test_msgs_TARGETS}
  )
  function(test_composable_recorder_for_rmw_implementation)
    set(rmw_implementation_env_var RMW_IMPLEMENTATION=${rmw_implementation})

    ament_add_gmock_test(test_composable_recorder
      TEST_NAME test_composable_recorder${target_suffix}
      ENV ${rmw_implementation_env_var}
    )
  endfunction()
  call_for_each_rmw_implementation(test_composable_recorder_for_rmw_implementation)

  ament_add_gmock_executable(test_load_composable_components test/rosbag2_transport/test_load_composable_components.cpp)
  target_link_libraries(test_load_composable_components
    rosbag2_transport
    ${test_msgs_TARGETS}
    rosbag2_test_common::rosbag2_test_common
    ${rclcpp_components_TARGETS}
  )
  function(test_load_composable_components_for_rmw_implementation)
    set(rmw_implementation_env_var RMW_IMPLEMENTATION=${rmw_implementation})

    ament_add_gmock_test(test_load_composable_components
      TEST_NAME test_load_composable_components${target_suffix}
      ENV ${rmw_implementation_env_var}
    )
  endfunction()
  call_for_each_rmw_implementation(test_load_composable_components_for_rmw_implementation)

  # Mark a few tests as xfail
  if(TARGET test_play_services__rmw_cyclonedds_cpp)
    ament_add_test_label(test_play_services__rmw_cyclonedds_cpp xfail)
  endif()
  if(TARGET test_play_services__rmw_fastrtps_cpp)
    ament_add_test_label(test_play_services__rmw_fastrtps_cpp xfail)
  endif()
  if(TARGET test_play_services__rmw_fastrtps_dynamic_cpp)
    ament_add_test_label(test_play_services__rmw_fastrtps_dynamic_cpp xfail)
  endif()

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
    rcpputils::rcpputils
    rosbag2_test_common::rosbag2_test_common
  )

  ament_add_gmock(test_recorder_event_notifier
    test/rosbag2_transport/test_recorder_event_notifier.cpp)
  target_link_libraries(test_recorder_event_notifier
    ${PROJECT_NAME}
    rosbag2_test_common::rosbag2_test_common
    ${rosbag2_interfaces_TARGETS}
  )

  ament_add_gmock(test_readers_manager test/rosbag2_transport/test_readers_manager.cpp)
  target_link_libraries(test_readers_manager
    ${PROJECT_NAME}
    rosbag2_test_common::rosbag2_test_common
    rcutils::rcutils
  )
endif()

ament_package()
