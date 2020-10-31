cmake_minimum_required(VERSION 3.7)

project(leveldb_vendor)

find_package(ament_cmake REQUIRED)

list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
find_package(leveldb QUIET)

if (NOT leveldb_FOUND)
  message( FATAL_ERROR "Not find leveldb, CMake will exit." )
else()
  message(STATUS "Found leveldb. Using leveldb from system.")
endif()

install(DIRECTORY cmake DESTINATION share/${PROJECT_NAME})

ament_package(CONFIG_EXTRAS leveldb_vendor-extras.cmake)
