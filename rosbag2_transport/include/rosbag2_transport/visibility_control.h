#ifndef ROSBAG2_TRANSPORT__VISIBILITY_CONTROL_H_
#define ROSBAG2_TRANSPORT__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSBAG2_TRANSPORT_EXPORT __attribute__ ((dllexport))
    #define ROSBAG2_TRANSPORT_IMPORT __attribute__ ((dllimport))
  #else
    #define ROSBAG2_TRANSPORT_EXPORT __declspec(dllexport)
    #define ROSBAG2_TRANSPORT_IMPORT __declspec(dllimport)
  #endif
  #ifdef ROSBAG2_TRANSPORT_BUILDING_LIBRARY
    #define ROSBAG2_TRANSPORT_PUBLIC ROSBAG2_TRANSPORT_EXPORT
  #else
    #define ROSBAG2_TRANSPORT_PUBLIC ROSBAG2_TRANSPORT_IMPORT
  #endif
  #define ROSBAG2_TRANSPORT_PUBLIC_TYPE ROSBAG2_TRANSPORT_PUBLIC
  #define ROSBAG2_TRANSPORT_LOCAL
#else
  #define ROSBAG2_TRANSPORT_EXPORT __attribute__ ((visibility("default")))
  #define ROSBAG2_TRANSPORT_IMPORT
  #if __GNUC__ >= 4
    #define ROSBAG2_TRANSPORT_PUBLIC __attribute__ ((visibility("default")))
    #define ROSBAG2_TRANSPORT_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ROSBAG2_TRANSPORT_PUBLIC
    #define ROSBAG2_TRANSPORT_LOCAL
  #endif
  #define ROSBAG2_TRANSPORT_PUBLIC_TYPE
#endif

#endif  // ROSBAG2_TRANSPORT__VISIBILITY_CONTROL_H_
