find_package(ament_cmake_python REQUIRED)
find_package(python_cmake_module REQUIRED)
find_package(PythonExtra MODULE REQUIRED)

set(_PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}")

if(WIN32 AND CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE_DEBUG}")
endif()

function(set_properties _targetname _build_type)

  set_target_properties(${_targetname} PROPERTIES
    PREFIX ""
    LIBRARY_OUTPUT_DIRECTORY${_build_type} "${CMAKE_CURRENT_BINARY_DIR}/test_${PROJECT_NAME}"
    RUNTIME_OUTPUT_DIRECTORY${_build_type} "${CMAKE_CURRENT_BINARY_DIR}/test_${PROJECT_NAME}"
    OUTPUT_NAME "_${_targetname}${PythonExtra_EXTENSION_SUFFIX}"
    SUFFIX "${PythonExtra_EXTENSION_EXTENSION}")
endfunction()

function(configure_python_c_extension_library _library_name)
  set_properties(${_library_name} "")
  if(WIN32)
    set_properties(${_library_name} "_DEBUG")
    set_properties(${_library_name} "_MINSIZEREL")
    set_properties(${_library_name} "_RELEASE")
    set_properties(${_library_name} "_RELWITHDEBINFO")
  endif()

  target_link_libraries(${_library_name}
    ${PythonExtra_LIBRARIES}
  )

  target_include_directories(${_library_name}
    PUBLIC
    ${PythonExtra_INCLUDE_DIRS}
  )

  install(TARGETS ${_library_name}
    DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
  )
endfunction()
