cmake_minimum_required(VERSION 3.5)
project(rosbag2_converter_default_plugins)

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

# Windows supplies macros for min and max by default. We should only use min and max from stl
if(WIN32)
  add_definitions(-DNOMINMAX)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rosbag2_compression REQUIRED)
find_package(zstd_vendor REQUIRED)
find_package(zstd REQUIRED)


add_library(zstd_compressor SHARED
  src/zstd_compressor.cpp)
ament_target_dependencies(zstd_compressor
  rosbag2_compression
  zstd_vendor
  pluginlib)
target_include_directories(zstd_compressor
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(zstd_compressor PRIVATE
  ROSBAG2_COMPRESSION_DEFAULT_PLUGINS_BUILDING_DLL)

add_library(zstd_decompressor SHARED
  src/zstd_decompressor.cpp)
ament_target_dependencies(zstd_decompressor
  rosbag2_compression
  zstd_vendor
  pluginlib)
target_include_directories(zstd_decompressor
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(zstd_compressor PRIVATE
  ROSBAG2_COMPRESSION_DEFAULT_PLUGINS_BUILDING_DLL)


pluginlib_export_plugin_description_file(rosbag2_compression plugin_description.xml)

install(
  DIRECTORY include/
  DESTINATION include)

install(
  TARGETS zstd_compressor zstd_decompressor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rosbag2_compression)


ament_package()
