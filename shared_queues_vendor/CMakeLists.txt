cmake_minimum_required(VERSION 3.5)
project(shared_queues_vendor)

find_package(ament_cmake REQUIRED)

# Avoid DOWNLOAD_EXTRACT_TIMESTAMP warning for CMake >= 3.24
if (POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW)
endif()

include(ExternalProject)
# Single producer single consumer queue by moodycamel - header only, don't build, install
ExternalProject_Add(ext-singleproducerconsumer
  PREFIX singleproducerconsumer
  DOWNLOAD_DIR ${CMAKE_CURRENT_BINARY_DIR}/download
  URL https://github.com/cameron314/readerwriterqueue/archive/ef7dfbf553288064347d51b8ac335f1ca489032a.zip
  URL_MD5 64c673dd381b8fae9254053ad7b2be4d
  TIMEOUT 60
  INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND ""
  )

add_library(singleproducerconsumer INTERFACE)
target_include_directories(singleproducerconsumer INTERFACE $<INSTALL_INTERFACE:include/moodycamel>)

install(
  TARGETS singleproducerconsumer
  EXPORT export_${PROJECT_NAME}
  INCLUDES DESTINATION include
)

# Install headers
install(
  FILES
  "${CMAKE_CURRENT_BINARY_DIR}/singleproducerconsumer/src/ext-singleproducerconsumer/atomicops.h"
  "${CMAKE_CURRENT_BINARY_DIR}/singleproducerconsumer/src/ext-singleproducerconsumer/readerwriterqueue.h"
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/moodycamel
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})
ament_package()
