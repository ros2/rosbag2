cmake_minimum_required(VERSION 3.14)
project(rosbag2_storage_mcap)

# Set Release build if no build type was specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING
      "Build type for the build. Possible values are: Debug, Release, RelWithDebInfo, MinSizeRel"
      FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
      "Debug" "Release" "RelWithDebInfo" "MinSizeRel")
endif()

# Enable additional warnings and warnings as errors
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Get the ROS_DISTRO environment variable
set(ROS_DISTRO $ENV{ROS_DISTRO})

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(mcap_vendor REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rcutils REQUIRED)
find_package(rosbag2_storage REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/mcap_storage.cpp
  src/message_definition_cache.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)
target_compile_definitions(${PROJECT_NAME} PRIVATE "ROSBAG2_STORAGE_MCAP_BUILDING_DLL")
ament_target_dependencies(${PROJECT_NAME}
  mcap_vendor
  pluginlib
  rcutils
  rosbag2_storage)

set(MCAP_COMPILE_DEFS)
# COMPATIBILITY(foxy) - 0.3.x is the Foxy release
if(${rosbag2_storage_VERSION} VERSION_GREATER_EQUAL 0.4.0)
  list(APPEND MCAP_COMPILE_DEFS ROSBAG2_STORAGE_MCAP_HAS_STORAGE_OPTIONS)
  list(APPEND MCAP_COMPILE_DEFS ROSBAG2_STORAGE_MCAP_WRITER_CREATES_DIRECTORY)
endif()
# COMPATIBILITY(galactic) - 0.9.x is the Galactic release
if(${rosbag2_storage_VERSION} VERSION_GREATER_EQUAL 0.10.0)
  list(APPEND MCAP_COMPILE_DEFS ROSBAG2_STORAGE_MCAP_OVERRIDE_SEEK_METHOD)
endif()
# COMPATIBILITY(foxy, galactic) - 0.15.x is the Humble release
if(${rosbag2_storage_VERSION} VERSION_GREATER_EQUAL 0.15.0)
  list(APPEND MCAP_COMPILE_DEFS ROSBAG2_STORAGE_MCAP_HAS_YAML_HPP)
endif()
# COMPATIBILITY(foxy, galactic, humble, rolling:0.17.x)
if(${rosbag2_storage_VERSION} VERSION_GREATER_EQUAL 0.17.0)
  list(APPEND MCAP_COMPILE_DEFS ROSBAG2_STORAGE_MCAP_HAS_STORAGE_FILTER_TOPIC_REGEX)
endif()
# COMPATIBILITY(foxy, galactic, humble, rolling:0.17.x, rolling:0.18.x)
if(${rosbag2_storage_VERSION} VERSION_GREATER_EQUAL 0.18.0)
  list(APPEND MCAP_COMPILE_DEFS ROSBAG2_STORAGE_MCAP_HAS_SET_READ_ORDER)
  list(APPEND MCAP_COMPILE_DEFS ROSBAG2_STORAGE_MCAP_HAS_UPDATE_METADATA)
endif()

target_compile_definitions(${PROJECT_NAME} PRIVATE ${MCAP_COMPILE_DEFS})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "ROSBAG2_STORAGE_MCAP_BUILDING_LIBRARY")

pluginlib_export_plugin_description_file(rosbag2_storage plugin_description.xml)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(rcpputils REQUIRED)
  find_package(rosbag2_test_common REQUIRED)
  find_package(std_msgs REQUIRED)

  add_definitions(-D_TEST_RESOURCES_DIR_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test/${PROJECT_NAME}")

  set(ament_cmake_clang_format_CONFIG_FILE .clang-format)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  if(WIN32)
    # clang-format is not easily exposed to Windows, this linter enforced regardless via Linux/OSX builds
    list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_clang_format)
  endif()
  ament_lint_auto_find_test_dependencies()

  ament_add_gmock(test_mcap_storage test/rosbag2_storage_mcap/test_mcap_storage.cpp)
  target_link_libraries(test_mcap_storage ${PROJECT_NAME})
  ament_target_dependencies(test_mcap_storage mcap_vendor rosbag2_storage rosbag2_test_common std_msgs)
  target_compile_definitions(test_mcap_storage PRIVATE ${MCAP_COMPILE_DEFS})

  ament_add_gmock(test_message_definition_cache test/rosbag2_storage_mcap/test_message_definition_cache.cpp)
  target_link_libraries(test_message_definition_cache ${PROJECT_NAME})
endif()


ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(rosbag2_storage rcutils)

ament_package()
