cmake_minimum_required(VERSION 3.7.2)
project(rosbag2)

set(CMAKE_CXX_CLANG_TIDY
    clang-tidy;
    -header-filter=.;
    -checks=google-*;)

find_package(ament_cmake REQUIRED)
ament_package()
