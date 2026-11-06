cmake_minimum_required(VERSION 3.20)

project(rosbag2_performance_benchmarking_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ByteArray.msg"
  DEPENDENCIES
)

# Keep Python extension object paths below the Windows limit without changing
# the generated module names. Compile each generated C file through a short wrapper.
get_property(_targets DIRECTORY PROPERTY BUILDSYSTEM_TARGETS)
foreach(_target IN LISTS _targets)
  if(_target MATCHES "^${PROJECT_NAME}_s__rosidl_typesupport_")
    get_target_property(_sources ${_target} SOURCES)
    set(_short_sources)
    foreach(_source IN LISTS _sources)
      string(MD5 _source_hash "${_source}")
      set(_wrapper "${CMAKE_CURRENT_BINARY_DIR}/${_source_hash}.c")
      file(GENERATE OUTPUT "${_wrapper}" CONTENT "#include \"${_source}\"\n")
      set_source_files_properties("${_wrapper}" PROPERTIES
        GENERATED TRUE
        OBJECT_DEPENDS "${_source}"
      )
      list(APPEND _short_sources "${_wrapper}")
    endforeach()
    set_property(TARGET ${_target} PROPERTY SOURCES "${_short_sources}")
  endif()
endforeach()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
