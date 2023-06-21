cmake_minimum_required(VERSION 3.10)
project(sqlite3_vendor)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
find_package(SQLite3 QUIET)

set(CMAKE_ARGS)
if(WIN32)
  list(APPEND CMAKE_ARGS -DCMAKE_WINDOWS_EXPORT_ALL_SYMBOLS:BOOL=ON)
endif()

ament_vendor(sqlite3_vendor
  SATISFIED ${SQLite3_FOUND}
  VCS_TYPE zip
  VCS_URL https://www.sqlite.org/2022/sqlite-amalgamation-3370200.zip
  VCS_VERSION sqlite-amalgamation-3370200
  CMAKE_ARGS ${CMAKE_ARGS}
  PATCHES patches
)

install(DIRECTORY cmake DESTINATION share/${PROJECT_NAME})

ament_package(
  CONFIG_EXTRAS "sqlite3_vendor-extras.cmake"
)
