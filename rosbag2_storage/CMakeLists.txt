cmake_minimum_required(VERSION 3.20)
project(rosbag2_storage)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Windows supplies macros for min and max by default. We should only use min and max from stl
if(WIN32)
  add_definitions(-DNOMINMAX)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rcutils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/rosbag2_storage/qos.cpp
  src/rosbag2_storage/default_storage_id.cpp
  src/rosbag2_storage/metadata_io.cpp
  src/rosbag2_storage/ros_helper.cpp
  src/rosbag2_storage/storage_factory.cpp
  src/rosbag2_storage/storage_options.cpp
  src/rosbag2_storage/base_io_interface.cpp)
target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  rcutils::rcutils
  rclcpp::rclcpp
  rmw::rmw
  yaml-cpp::yaml-cpp
)
target_link_libraries(${PROJECT_NAME} PRIVATE
  ament_cmake_ros_core::ament_ros_defaults
  pluginlib::pluginlib
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "ROSBAG2_STORAGE_BUILDING_DLL")

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Export old-style CMake variables
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})

# Export modern CMake targets
ament_export_targets(export_${PROJECT_NAME})

ament_export_dependencies(rclcpp rcutils rmw yaml-cpp yaml_cpp_vendor)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(rosbag2_test_common REQUIRED)

  ament_lint_auto_find_test_dependencies()

  add_library(
    test_plugin
    SHARED
    test/rosbag2_storage/test_plugin.cpp
    test/rosbag2_storage/test_read_only_plugin.cpp)
  target_link_libraries(test_plugin ament_cmake_ros_core::ament_ros_defaults ${PROJECT_NAME} pluginlib::pluginlib)
  # Mock-install the test plugin into the build folder rather than installing
  # it, so that it never appears as a registered storage plugin outside of the
  # tests that need it.
  pluginlib_enable_plugin_testing(
    CMAKE_TARGET_VAR test_plugin_mock_install_target
    AMENT_PREFIX_PATH_VAR test_plugin_mock_install_path
    PACKAGE_NAME "rosbag2_storage_test_plugins"
    PACKAGE_XML "test/rosbag2_storage/test_package.xml"
    PLUGIN_CATEGORY "rosbag2_storage"
    PLUGIN_DESCRIPTIONS "test/rosbag2_storage/test_plugin.xml"
    PLUGIN_LIBRARIES test_plugin)

  ament_add_gmock(test_storage_factory
    test/rosbag2_storage/test_storage_factory.cpp
    APPEND_LIBRARY_DIRS "$<TARGET_FILE_DIR:test_plugin>"
    APPEND_ENV AMENT_PREFIX_PATH=${test_plugin_mock_install_path})
  if(TARGET test_storage_factory)
    target_link_libraries(test_storage_factory ament_cmake_ros_core::ament_ros_defaults ${PROJECT_NAME})
    add_dependencies(test_storage_factory "${test_plugin_mock_install_target}")
  endif()

  ament_add_gmock(test_ros_helper
    test/rosbag2_storage/test_ros_helper.cpp)
  if(TARGET test_ros_helper)
    target_link_libraries(test_ros_helper ament_cmake_ros_core::ament_ros_defaults ${PROJECT_NAME})
  endif()

  ament_add_gmock(test_metadata_serialization
    test/rosbag2_storage/test_metadata_serialization.cpp)
  if(TARGET test_metadata_serialization)
    target_link_libraries(test_metadata_serialization
      ament_cmake_ros_core::ament_ros_defaults
      ${PROJECT_NAME}
      rosbag2_test_common::rosbag2_test_common
    )
  endif()

  ament_add_gmock(test_storage_options
    test/rosbag2_storage/test_storage_options.cpp)
  target_link_libraries(test_storage_options
    ament_cmake_ros_core::ament_ros_defaults
    ${PROJECT_NAME}
    rosbag2_test_common::rosbag2_test_common
  )

  ament_add_gmock(test_qos
    test/rosbag2_storage/test_qos.cpp)
  target_link_libraries(test_qos
    ament_cmake_ros_core::ament_ros_defaults
    ${PROJECT_NAME}
    rosbag2_test_common::rosbag2_test_common
    yaml-cpp::yaml-cpp
  )
endif()

ament_package()
