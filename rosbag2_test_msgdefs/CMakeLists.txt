cmake_minimum_required(VERSION 3.20)
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
  "msg/nested_sub_dir/AnotherBasicMsg.msg"
  "msg/ComplexMsg.msg"
  "msg/ComplexMsgDependsOnIdl.msg"
  "srv/BasicSrv.srv"
  "srv/ComplexSrvMsg.srv"
  "srv/ComplexSrvIdl.srv"
  "action/BasicAction.action"
  "nested_sub_dir/action/BasicMsg.action"
  "action/ComplexActionMsg.action"
  "action/ComplexActionIdl.action"
  ADD_LINTER_TESTS
)

ament_package()
