cmake_minimum_required(VERSION 3.5)
project(shared_queues_vendor)

find_package(ament_cmake REQUIRED)

include(ExternalProject)
# Single producer single consumer queue by moodycamel - header only, don't build, install
ExternalProject_Add(singleproducerconsumer
  DOWNLOAD_DIR ${CMAKE_CURRENT_BINARY_DIR}/single
  URL https://github.com/cameron314/readerwriterqueue/archive/ef7dfbf553288064347d51b8ac335f1ca489032a.zip
  URL_MD5 64c673dd381b8fae9254053ad7b2be4d
  TIMEOUT 60
  INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND ""
  )

# Concurrent and blocking concurrent queue by moodycamel - header only, don't build, install
ExternalProject_Add(concurrentqueue
  DOWNLOAD_DIR ${CMAKE_CURRENT_BINARY_DIR}/multiple
  URL https://github.com/cameron314/concurrentqueue/archive/8f65a8734d77c3cc00d74c0532efca872931d3ce.zip
  URL_MD5 71a0d932cc89150c2ade85f0d9cac9dc
  TIMEOUT 60
  INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND ""
  )

# Install headers
install(
  FILES
  "${CMAKE_CURRENT_BINARY_DIR}/singleproducerconsumer-prefix/src/singleproducerconsumer/atomicops.h"
  "${CMAKE_CURRENT_BINARY_DIR}/singleproducerconsumer-prefix/src/singleproducerconsumer/readerwriterqueue.h"
  "${CMAKE_CURRENT_BINARY_DIR}/concurrentqueue-prefix/src/concurrentqueue/concurrentqueue.h"
  "${CMAKE_CURRENT_BINARY_DIR}/concurrentqueue-prefix/src/concurrentqueue/blockingconcurrentqueue.h"
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/moodycamel
)

ament_export_include_directories(include)

ament_package()