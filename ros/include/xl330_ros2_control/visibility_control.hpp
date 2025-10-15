#ifndef XL330_ROS2_CONTROL__VISIBILITY_CONTROL_HPP_
#define XL330_ROS2_CONTROL__VISIBILITY_CONTROL_HPP_

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define XL330_ROS2_CONTROL_EXPORT __attribute__ ((dllexport))
    #define XL330_ROS2_CONTROL_IMPORT __attribute__ ((dllimport))
  #else
    #define XL330_ROS2_CONTROL_EXPORT __declspec(dllexport)
    #define XL330_ROS2_CONTROL_IMPORT __declspec(dllimport)
  #endif
  #ifdef XL330_ROS2_CONTROL_BUILDING_DLL
    #define XL330_ROS2_CONTROL_PUBLIC XL330_ROS2_CONTROL_EXPORT
  #else
    #define XL330_ROS2_CONTROL_PUBLIC XL330_ROS2_CONTROL_IMPORT
  #endif
  #define XL330_ROS2_CONTROL_PUBLIC_TYPE XL330_ROS2_CONTROL_PUBLIC
  #define XL330_ROS2_CONTROL_LOCAL
#else
  #define XL330_ROS2_CONTROL_EXPORT __attribute__ ((visibility("default")))
  #define XL330_ROS2_CONTROL_IMPORT
  #if __GNUC__ >= 4
    #define XL330_ROS2_CONTROL_PUBLIC __attribute__ ((visibility("default")))
    #define XL330_ROS2_CONTROL_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define XL330_ROS2_CONTROL_PUBLIC
    #define XL330_ROS2_CONTROL_LOCAL
  #endif
  #define XL330_ROS2_CONTROL_PUBLIC_TYPE
#endif

#endif  // XL330_ROS2_CONTROL__VISIBILITY_CONTROL_HPP_
