cmake_minimum_required(VERSION 3.5)
project(mcap_vendor LANGUAGES C CXX ASM)

## Dependencies
find_package(ament_cmake REQUIRED)
find_package(zstd_vendor REQUIRED)
find_package(zstd REQUIRED)

## Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()
if (MSVC)
  add_compile_options(/W4 /WX
    /wd4251 # suppress warning about having template instances (such as std::string) as public class members
  )
  # suppress error disallowing `fopen` in favor of `fopen_s`
  add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
endif()

## Define vendor macro
macro(build_mcap_vendor)
  include(FetchContent)
  fetchcontent_declare(mcap
    URL https://github.com/foxglove/mcap/archive/refs/tags/releases/cpp/v0.7.0.tar.gz
    URL_HASH SHA1=0fccc7bf49e3d8f4dc05219f537cf7d9fa4adb42)
  fetchcontent_makeavailable(mcap)

  fetchcontent_declare(lz4
    GIT_REPOSITORY https://github.com/lz4/lz4.git
    GIT_TAG d44371841a2f1728a3f36839fd4b7e872d0927d3 # v1.9.3
  )
  fetchcontent_makeavailable(lz4)

  file(GLOB _lz4_srcs
    ${lz4_SOURCE_DIR}/lib/*.c)

  add_library(mcap SHARED
    src/main.cpp
    ${_lz4_srcs}
  )

  set(_mcap_include_dir ${mcap_SOURCE_DIR}/cpp/mcap/include)

  target_include_directories(mcap PRIVATE
    ${lz4_SOURCE_DIR}/lib
  )
  target_include_directories(mcap PUBLIC
    "$<BUILD_INTERFACE:${_mcap_include_dir}>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  )
  ament_target_dependencies(mcap zstd)

  install(
    DIRECTORY ${_mcap_include_dir}/mcap
    DESTINATION include/${PROJECT_NAME}
  )

  install(
    TARGETS mcap
    EXPORT mcap
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
endmacro()

## Call vendor macro
build_mcap_vendor()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_targets(mcap HAS_LIBRARY_TARGET)
ament_export_dependencies(zstd_vendor zstd)

## Package
ament_package()
