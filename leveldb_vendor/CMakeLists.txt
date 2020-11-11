cmake_minimum_required(VERSION 3.7)

project(leveldb_vendor)

find_package(ament_cmake REQUIRED)
find_package(snappy_vendor REQUIRED)
find_package(snappy REQUIRED)

string(REPLACE "include" "" snappy_install_path ${snappy_INCLUDE_DIR})

list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
find_package(leveldb QUIET)

if (NOT leveldb_FOUND)
  set(extra_cmake_args)

  if(DEFINED CMAKE_TOOLCHAIN_FILE)
    list(APPEND extra_cmake_args "-DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}")
  endif()

  find_package(Patch REQUIRED)
  include(ExternalProject)
  ExternalProject_Add(leveldb-1.22
    PREFIX leveldb-1.22
    URL https://github.com/google/leveldb/archive/1.22.zip
    URL_MD5 f741bc416308adb35d79900afe282d9e
    CMAKE_ARGS
      -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/leveldb_install
      -DBUILD_SHARED_LIBS:BOOL=ON
      -DLEVELDB_BUILD_TESTS:BOOL=OFF
      -DLEVELDB_BUILD_BENCHMARKS:BOOL=OFF
      -DSNAPPY_INSTALL_PATH:STRING=${snappy_install_path}
      ${extra_cmake_args}
    PATCH_COMMAND
      ${Patch_EXECUTABLE} -p1 -N < ${CMAKE_CURRENT_SOURCE_DIR}/leveldb_cmakefile.diff
    )
  install(
    DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/leveldb_install/
    DESTINATION ${CMAKE_INSTALL_PREFIX})
else()
  message(STATUS "Found leveldb. Using leveldb from system.")
endif()

install(DIRECTORY cmake DESTINATION share/${PROJECT_NAME})

ament_package(CONFIG_EXTRAS leveldb_vendor-extras.cmake)
