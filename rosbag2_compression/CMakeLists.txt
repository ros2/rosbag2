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
find_package(rcutils REQUIRED)
find_package(rosbag2_storage REQUIRED)

install(
  DIRECTORY include/
  DESTINATION include)

ament_export_include_directories(include)
ament_export_dependencies(rcutils rosbag2_storage)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gmock(test_compressor
    test/rosbag2_compression/test_compressor.cpp
    test/rosbag2_compression/fake_compressor.cpp)
  target_include_directories(test_compressor PUBLIC include)
  ament_target_dependencies(test_compressor rosbag2_storage)
endif()

ament_package()
