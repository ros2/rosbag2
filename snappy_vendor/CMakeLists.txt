cmake_minimum_required(VERSION 3.7)

project(snappy_vendor)

find_package(ament_cmake REQUIRED)

list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
find_package(snappy QUIET)

if (NOT snappy_FOUND)
  set(extra_cmake_args)

  if(DEFINED CMAKE_TOOLCHAIN_FILE)
    list(APPEND extra_cmake_args "-DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}")
  endif()

  include(ExternalProject)
  ExternalProject_Add(snappy-1.1.8
    PREFIX snappy-1.1.8
    URL https://github.com/google/snappy/archive/1.1.8.zip
    URL_MD5 2fe0949a4285ebfe63651d06c642f4fa
    TIMEOUT 120
    CMAKE_ARGS
      -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/snappy_install
      -DBUILD_SHARED_LIBS:BOOL=ON
      -DSNAPPY_BUILD_TESTS:BOOL=OFF
      ${extra_cmake_args}
    )

  install(
    DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/snappy_install/
    DESTINATION ${CMAKE_INSTALL_PREFIX})

else()
  message(STATUS "Found snappy. Using snappy from system.")
endif()

install(DIRECTORY cmake DESTINATION share/${PROJECT_NAME})

ament_package(CONFIG_EXTRAS snappy_vendor-extras.cmake)
