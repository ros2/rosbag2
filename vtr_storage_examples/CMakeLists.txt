cmake_minimum_required(VERSION 3.5)
project(vtr_storage_examples)

include("${CMAKE_CURRENT_LIST_DIR}/../../vtr_common/vtr_include.cmake.in")

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(vtr_storage REQUIRED)
find_package(test_msgs REQUIRED)
# find_package(std_msgs REQUIRED)

  
add_executable(sample_node_2 src/sample_node_2.cpp)
target_include_directories(sample_node_2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(sample_node_3 src/sample_node_3.cpp)
target_include_directories(sample_node_3 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  
ament_target_dependencies(sample_node_2
      vtr_storage
      test_msgs)
      
ament_target_dependencies(sample_node_3
      vtr_storage
      test_msgs)

install(TARGETS sample_node_2
  DESTINATION lib/${PROJECT_NAME})
 
install(TARGETS sample_node_3
  DESTINATION lib/${PROJECT_NAME})

ament_package()
