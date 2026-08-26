cmake_minimum_required(VERSION 3.20)
project(rosbag2_compression)

# Windows supplies macros for min and max by default. We should only use min and max from stl
if(WIN32)
  add_definitions(-DNOMINMAX)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rcpputils REQUIRED)
find_package(rcutils REQUIRED)
find_package(rosbag2_cpp)
find_package(rosbag2_storage REQUIRED)

add_library(${PROJECT_NAME}
  SHARED
  src/rosbag2_compression/compression_factory.cpp
  src/rosbag2_compression/compression_options.cpp
  src/rosbag2_compression/sequential_compression_reader.cpp
  src/rosbag2_compression/sequential_compression_writer.cpp)
target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(${PROJECT_NAME}
  ament_cmake_ros_core::ament_ros_defaults
  pluginlib::pluginlib
  rcpputils::rcpputils
  rcutils::rcutils
  rosbag2_cpp::rosbag2_cpp
  rosbag2_storage::rosbag2_storage
)
target_compile_definitions(${PROJECT_NAME} PRIVATE ROSBAG2_COMPRESSION_BUILDING_DLL)

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

ament_export_dependencies(pluginlib rcpputils rcutils rosbag2_cpp rosbag2_storage)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rosbag2_test_common REQUIRED)
  find_package(test_msgs REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_library(fake_plugin SHARED
    test/rosbag2_compression/fake_compressor.cpp
    test/rosbag2_compression/fake_decompressor.cpp)
  target_link_libraries(fake_plugin ament_cmake_ros_core::ament_ros_defaults ${PROJECT_NAME})
  # Mock-install the fake compression plugin into the build folder rather
  # than installing it, so that it never appears as a registered compression
  # plugin outside of the tests that need it.
  pluginlib_enable_plugin_testing(
    CMAKE_TARGET_VAR fake_plugin_mock_install_target
    AMENT_PREFIX_PATH_VAR fake_plugin_mock_install_path
    PACKAGE_NAME "rosbag2_compression_test_plugins"
    PACKAGE_XML "test/rosbag2_compression/test_package.xml"
    PLUGIN_CATEGORY "rosbag2_compression"
    PLUGIN_DESCRIPTIONS "test/rosbag2_compression/fake_plugin.xml"
    PLUGIN_LIBRARIES fake_plugin)

  ament_add_gmock(test_compression_factory
    test/rosbag2_compression/test_compression_factory.cpp
    APPEND_LIBRARY_DIRS "$<TARGET_FILE_DIR:fake_plugin>"
    APPEND_ENV AMENT_PREFIX_PATH=${fake_plugin_mock_install_path})
  target_link_libraries(test_compression_factory ament_cmake_ros_core::ament_ros_defaults ${PROJECT_NAME})
  add_dependencies(test_compression_factory "${fake_plugin_mock_install_target}")

  ament_add_gmock(test_compression_options
    test/rosbag2_compression/test_compression_options.cpp)
  target_link_libraries(test_compression_options ament_cmake_ros_core::ament_ros_defaults ${PROJECT_NAME})

  ament_add_gmock(test_sequential_compression_reader
    test/rosbag2_compression/test_sequential_compression_reader.cpp
    APPEND_LIBRARY_DIRS "$<TARGET_FILE_DIR:fake_plugin>"
    APPEND_ENV AMENT_PREFIX_PATH=${fake_plugin_mock_install_path})
  target_link_libraries(test_sequential_compression_reader
    ament_cmake_ros_core::ament_ros_defaults
    ${PROJECT_NAME}
    rosbag2_cpp::rosbag2_cpp
    rosbag2_storage::rosbag2_storage
  )
  add_dependencies(test_sequential_compression_reader "${fake_plugin_mock_install_target}")

  ament_add_gmock(test_sequential_compression_writer
    test/rosbag2_compression/test_sequential_compression_writer.cpp
    APPEND_LIBRARY_DIRS "$<TARGET_FILE_DIR:fake_plugin>"
    APPEND_ENV AMENT_PREFIX_PATH=${fake_plugin_mock_install_path})
  target_link_libraries(test_sequential_compression_writer
    ament_cmake_ros_core::ament_ros_defaults
    ${PROJECT_NAME}
    fake_plugin
    rosbag2_cpp::rosbag2_cpp
    rosbag2_storage::rosbag2_storage
  )
  add_dependencies(test_sequential_compression_writer "${fake_plugin_mock_install_target}")
endif()

ament_package()
