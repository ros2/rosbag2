cmake_minimum_required(VERSION 3.8)
project(rosbag2_test_msgdefs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ComplexIdl.idl"
  "msg/BasicIdl.idl"
  "msg/BasicMsg.msg"
  "msg/ComplexMsg.msg"
  "msg/ComplexMsgDependsOnIdl.msg"
  "srv/BasicSrv.srv"
  ADD_LINTER_TESTS
)

ament_package()
