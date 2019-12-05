cmake_minimum_required(VERSION 3.5)
project(zstd_vendor)

list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
find_package(zstd)

if (NOT zstd_FOUND)
  set(extra_cmake_args)

  if(DEFINED CMAKE_TOOLCHAIN_FILE)
    list(APPEND extra_cmake_args "-DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}")
  endif()

  include(ExternalProject)
  set(EP_PREFIX zstd-1.4.4)
  ExternalProject_Add(zstd-1.4.4
    GIT_REPOSITORY https://github.com/facebook/zstd.git
    GIT_TAG v1.4.4
    CONFIGURE_COMMAND ${CMAKE_COMMAND} ${CMAKE_CURRENT_BINARY_DIR}/zstd/src/${EP_PREFIX}/build/cmake
    CMAKE_ARGS
      -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/zstd_install
      -DCMAKE_BUILD_TYPE=Release
      ${extra_cmake_args})
    # INSTALL_DIR "${CMAKE_CURRENT_BINARY_DIR}/zstd_install")

  # The external project will install to the build folder, but we'll install that on make install.
  install(
    DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/zstd_install/ 
    DESTINATION ${CMAKE_INSTALL_PREFIX})
else ()
  message(STATUS "Found Zstd. Using Zstd from system.")
endif ()

configure_file(zstd_vendorConfig.cmake.in "${PROJECT_BINARY_DIR}/zstd_vendorConfig.cmake" @ONLY)
configure_file(zstd_vendorConfig-version.cmake.in "${PROJECT_BINARY_DIR}/zstd_vendorConfig-version.cmake" @ONLY)

install(
  DIRECTORY cmake 
  DESTINATION share/${PROJECT_NAME})

install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME})

install(FILES
  "${PROJECT_BINARY_DIR}/zstd_vendorConfig.cmake"
  "${PROJECT_BINARY_DIR}/zstd_vendorConfig-version.cmake"
  DESTINATION share/${PROJECT_NAME}/cmake)
