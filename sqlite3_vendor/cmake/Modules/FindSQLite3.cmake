##################################################################################################
#
# CMake script for finding SQLite3.
#
# Input variables:
#
# - SQLite3_ROOT_DIR (optional): When specified, header files and libraries will be searched for in
#     ${SQLite3_ROOT_DIR}/include
#     ${SQLite3_ROOT_DIR}/libs
#   respectively, and the default CMake search order will be ignored. When unspecified, the default
#   CMake search order is used.
#   This variable can be specified either as a CMake or environment variable. If both are set,
#   preference is given to the CMake variable.
#   Use this variable for finding packages installed in a nonstandard location, or for enforcing
#   that one of multiple package installations is picked up.
#
#
# Cache variables (not intended to be used in CMakeLists.txt files)
#
# - SQLite3_INCLUDE_DIR: Absolute path to package headers.
# - SQLite3_LIBRARY: Absolute path to library.
#
#
# Output variables:
#
# - SQLite3_FOUND: Boolean that indicates if the package was found
# - SQLite3_INCLUDE_DIRS: Paths to the necessary header files
# - SQLite3_LIBRARIES: Package libraries
#
#
# Example usage:
#
#  find_package(SQLite3)
#  if(NOT SQLite3_FOUND)
#    # Error handling
#  endif()
#  ...
#  include_directories(${SQLite3_INCLUDE_DIRS} ...)
#  ...
#  target_link_libraries(my_target ${SQLite3_LIBRARIES})
#
##################################################################################################

# First attempt to find Config-file package
find_package(SQLite3 CONFIG QUIET)
if (SQLite3_FOUND)
    message("Found SQLite3 from installed package in ${SQLite3_DIR}")
    # https://cmake.org/cmake/help/latest/module/FindSQLite3.html
    list(APPEND SQLite3_TARGETS SQLite::SQLite3)
else()
    # Get package location hint from environment variable (if any)
    if(NOT SQLite3_ROOT_DIR AND DEFINED ENV{SQLite3_ROOT_DIR})
        set(SQLite3_ROOT_DIR "$ENV{SQLite3_ROOT_DIR}" CACHE PATH
                "SQLite3 base directory location (optional, used for nonstandard installation paths)")
    endif()

    # Search path for nonstandard package locations
    if(SQLite3_ROOT_DIR)
        set(SQLite3_INCLUDE_PATH PATHS "${SQLite3_ROOT_DIR}/include" NO_DEFAULT_PATH)
        set(SQLite3_LIBRARY_PATH PATHS "${SQLite3_ROOT_DIR}/lib"     NO_DEFAULT_PATH)
    else()
        set(SQLite3_INCLUDE_PATH "")
        set(SQLite3_LIBRARY_PATH "")
    endif()

    # Find headers and libraries
    find_path(SQLite3_INCLUDE_DIR NAMES sqlite3.h PATH_SUFFIXES "SQLite3" ${SQLite3_INCLUDE_PATH})
    find_library(SQLite3_LIBRARY  NAMES sqlite3   PATH_SUFFIXES "SQLite3" ${SQLite3_LIBRARY_PATH})

    mark_as_advanced(SQLite3_INCLUDE_DIR SQLite3_LIBRARY)

    # Output variables generation
    include(FindPackageHandleStandardArgs)
    find_package_handle_standard_args(SQLite3 DEFAULT_MSG SQLite3_LIBRARY SQLite3_INCLUDE_DIR)

    set(SQLite3_FOUND ${SQLITE3_FOUND}) # Enforce case-correctness: Set appropriately cased variable...
    unset(SQLITE3_FOUND) # ...and unset uppercase variable generated by find_package_handle_standard_args

    if(SQLite3_FOUND)
        set(SQLite3_INCLUDE_DIRS ${SQLite3_INCLUDE_DIR})
        set(SQLite3_LIBRARIES ${SQLite3_LIBRARY})

        add_library(SQLite::SQLite3 UNKNOWN IMPORTED)
        set_property(TARGET SQLite::SQLite3 PROPERTY IMPORTED_LOCATION ${SQLite3_LIBRARY})
        set_property(TARGET SQLite::SQLite3 PROPERTY INCLUDE_DIRECTORIES ${SQLite3_INCLUDE_DIR})
        list(APPEND SQLite3_TARGETS SQLite::SQLite3)
    else()
        message(FATAL_ERROR "Unable to find SQLite3")
    endif()
endif()
