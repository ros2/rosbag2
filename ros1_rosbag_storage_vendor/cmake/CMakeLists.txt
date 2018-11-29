# rosbag_storage does not contain a toplevel CMakeLists so colcon gets confused
# This is also an opportunity, because we want to only build rosbag_storage anyways
cmake_minimum_required(VERSION 3.5)
add_subdirectory(tools/rosbag_storage)