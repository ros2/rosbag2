cmake_minimum_required(VERSION 3.5)
project(rosbag2_compression)

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

option(DISABLE_SANITIZERS "Disable the use of gcc sanitizers")
# Only enable sanitizers on x64 architectures
# https://github.com/google/sanitizers/issues/794
if(NOT ${CMAKE_SIZEOF_VOID_P} EQUAL 8)
  set(DISABLE_SANITIZERS ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rcpputils REQUIRED)
find_package(rcutils REQUIRED)
find_package(rosbag2_cpp)
find_package(rosbag2_storage REQUIRED)
find_package(zstd_vendor REQUIRED)

add_library(${PROJECT_NAME}_zstd
  SHARED
  src/rosbag2_compression/zstd_compressor.cpp
  src/rosbag2_compression/zstd_decompressor.cpp)
target_include_directories(${PROJECT_NAME}_zstd
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${zstd_vendor_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_zstd ${zstd_vendor_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME}_zstd
  rcpputils
  rcutils
  rosbag2_storage)
# Causes the visibility macros to use dllexport rather than dllimport.
# This is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME}_zstd
  PRIVATE
  ROSBAG2_COMPRESSION_BUILDING_DLL)

add_library(${PROJECT_NAME}
  SHARED
  src/rosbag2_compression/compression_options.cpp
  src/rosbag2_compression/sequential_compression_reader.cpp
  src/rosbag2_compression/sequential_compression_writer.cpp)
target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_zstd)
ament_target_dependencies(${PROJECT_NAME}
  rcpputils
  rcutils
  rosbag2_cpp
  rosbag2_storage)
target_compile_definitions(${PROJECT_NAME} PRIVATE ROSBAG2_COMPRESSION_BUILDING_DLL)

install(
  DIRECTORY include/
  DESTINATION include)

install(
  TARGETS ${PROJECT_NAME}_zstd
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_zstd)
ament_export_dependencies(rosbag2_storage rcutils zstd_vendor)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(rosbag2_test_common REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_library(fake_compressor SHARED
    test/rosbag2_compression/fake_compressor.cpp)
  target_include_directories(fake_compressor PUBLIC include)
  ament_target_dependencies(fake_compressor rosbag2_storage)
  set_target_properties(fake_compressor
    PROPERTIES
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN 1)
  target_compile_definitions(fake_compressor PRIVATE ROSBAG2_COMPRESSION_BUILDING_DLL)

  ament_add_gmock(test_compressor
    test/rosbag2_compression/test_compressor.cpp)
  target_link_libraries(test_compressor fake_compressor)
  ament_target_dependencies(test_compressor rcpputils)

  add_library(fake_decompressor SHARED
    test/rosbag2_compression/fake_decompressor.cpp)
  target_include_directories(fake_decompressor PUBLIC include)
  ament_target_dependencies(fake_decompressor rosbag2_storage rcpputils)
  set_target_properties(fake_decompressor
    PROPERTIES
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN 1)
  target_compile_definitions(fake_decompressor PRIVATE ROSBAG2_COMPRESSION_BUILDING_DLL)

  ament_add_gmock(test_decompressor
    test/rosbag2_compression/test_decompressor.cpp)
  target_link_libraries(test_decompressor fake_decompressor)

  ament_add_gmock(test_zstd_compressor
    test/rosbag2_compression/test_zstd_compressor.cpp)
  target_include_directories(test_zstd_compressor PUBLIC include)
  target_link_libraries(test_zstd_compressor ${PROJECT_NAME}_zstd)
  ament_target_dependencies(test_zstd_compressor rosbag2_test_common rosbag2_storage)

  ament_add_gmock(test_compression_options
    test/rosbag2_compression/test_compression_options.cpp)
  target_include_directories(test_compression_options PUBLIC include)
  target_link_libraries(test_compression_options ${PROJECT_NAME})

  ament_add_gmock(test_sequential_compression_reader
    test/rosbag2_compression/test_sequential_compression_reader.cpp)
  target_include_directories(test_sequential_compression_reader PUBLIC include)
  target_link_libraries(test_sequential_compression_reader ${PROJECT_NAME})
  ament_target_dependencies(test_sequential_compression_reader rosbag2_cpp)

  ament_add_gmock(test_sequential_compression_writer
    test/rosbag2_compression/test_sequential_compression_writer.cpp)
  target_include_directories(test_sequential_compression_writer PUBLIC include)
  target_link_libraries(test_sequential_compression_writer ${PROJECT_NAME})
  ament_target_dependencies(test_sequential_compression_writer rosbag2_cpp)
endif()

ament_package()
