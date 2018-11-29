cmake_minimum_required(VERSION 3.5)
project(ros1_rosbag_storage_vendor)

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

# find dependencies
find_package(ament_cmake REQUIRED)

# We don't need to build on Windows for now
if(WIN32)
  return()
endif()

if(WIN32)
  set(patch_exe patch.exe)
else()
  set(patch_exe patch)
endif()

include(ExternalProject)
ExternalProject_Add(ros1_rosbag_storage
  DOWNLOAD_DIR ${CMAKE_CURRENT_BINARY_DIR}/download
  URL https://github.com/ros/ros_comm/archive/melodic-devel.zip
  URL_MD5 cfa0b16433e119616e28ed26be9712c2
  TIMEOUT 60
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/rosbag_install
  PATCH_COMMAND ${patch_exe} -p1 -N < ${CMAKE_CURRENT_SOURCE_DIR}/rosbag1_patch.diff
)

install(
  DIRECTORY
    ${CMAKE_CURRENT_BINARY_DIR}/rosbag_install/
  DESTINATION
    ${CMAKE_INSTALL_PREFIX}/
)

ament_package()
