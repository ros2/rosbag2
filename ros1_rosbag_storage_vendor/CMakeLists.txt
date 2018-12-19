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
  message("Replay ROS1 rosbags is currently not supported on Windows systems")
  return()
endif()

include(ExternalProject)
# We have to include a number of patches to the rosbag
# 1. rosbag1_encryption_patch includes https://github.com/ros/ros_comm/pull/1499, without it, pluginlib won't link
# 2. The next two patches patch the pluginlib usage:
#    Missing boost/bind include
#    Using pluginlib with boost results in compile errors on some systems, so we need to switch to pluginlib without boost
# 3. The next patches patch the catkin package to ament and renames it
#    Patching to ament is necessary as otherwise colcon needs to build a catkin package together with an ament package, which is impossible
#    Renaming the package is necessary on some systems to not confuse the linker
#
# Note that we need to build with CMAKE_NO_SYSTEM_FROM_IMPORTED. This prohibits cmake from
# declaring any paths as "internal" (flagged with -isystem instead of -I) which can result in this
# vendor package building against the wrong pluginlib, because the paths get switched.
# Symptoms of wrong paths: The build of the rosbag for ROS 1 plugins fails with missing symbols
# like ros::package::getPlugins(...) when linking the vendor package
ExternalProject_Add(ros1_rosbag_storage
  DOWNLOAD_DIR ${CMAKE_CURRENT_BINARY_DIR}/download
  URL https://github.com/ros/ros_comm/archive/669fbd32d2f92cc295f4b024fcb2f982fddec0f0.zip
  URL_MD5 4c8b4c33165b223870f5d77bb697bef6
  TIMEOUT 60
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/rosbag_install -DCMAKE_NO_SYSTEM_FROM_IMPORTED=1
  PATCH_COMMAND
    patch -p1 -N < ${CMAKE_CURRENT_SOURCE_DIR}/resources/rosbag1_encryption_patch.diff &&
    patch -p1 -N < ${CMAKE_CURRENT_SOURCE_DIR}/resources/bagh.diff &&
    patch -p1 -N < ${CMAKE_CURRENT_SOURCE_DIR}/resources/bagcpp.diff &&
    patch -p1 -N < ${CMAKE_CURRENT_SOURCE_DIR}/resources/bagcpp_name.diff &&
    patch -p1 -N < ${CMAKE_CURRENT_SOURCE_DIR}/resources/plugin_descriptionxml.diff &&
    patch -p1 -N < ${CMAKE_CURRENT_SOURCE_DIR}/resources/packagexml.diff &&
    patch -p1 -N < ${CMAKE_CURRENT_SOURCE_DIR}/resources/cmakeliststxt.diff &&
    ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/cmake/CMakeLists.txt.in ${CMAKE_CURRENT_BINARY_DIR}/ros1_rosbag_storage-prefix/src/ros1_rosbag_storage/CMakeLists.txt
)

install(
  DIRECTORY
    ${CMAKE_CURRENT_BINARY_DIR}/rosbag_install/
  DESTINATION
    ${CMAKE_INSTALL_PREFIX}/
)

ament_package()
