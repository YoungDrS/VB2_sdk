#ifndef TAIHU_DUAL_ARM_HARDWARE__VISIBILITY_CONTROL_H_
#define TAIHU_DUAL_ARM_HARDWARE__VISIBILITY_CONTROL_H_

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define TAIHU_DUAL_ARM_HARDWARE_EXPORT __attribute__ ((dllexport))
    #define TAIHU_DUAL_ARM_HARDWARE_IMPORT __attribute__ ((dllimport))
  #else
    #define TAIHU_DUAL_ARM_HARDWARE_EXPORT __declspec(dllexport)
    #define TAIHU_DUAL_ARM_HARDWARE_IMPORT __declspec(dllimport)
  #endif
  #ifdef TAIHU_DUAL_ARM_HARDWARE_BUILDING_LIBRARY
    #define TAIHU_DUAL_ARM_HARDWARE_PUBLIC TAIHU_DUAL_ARM_HARDWARE_EXPORT
  #else
    #define TAIHU_DUAL_ARM_HARDWARE_PUBLIC TAIHU_DUAL_ARM_HARDWARE_IMPORT
  #endif
  #define TAIHU_DUAL_ARM_HARDWARE_PUBLIC_TYPE TAIHU_DUAL_ARM_HARDWARE_PUBLIC
  #define TAIHU_DUAL_ARM_HARDWARE_LOCAL
#else
  #define TAIHU_DUAL_ARM_HARDWARE_EXPORT __attribute__ ((visibility("default")))
  #define TAIHU_DUAL_ARM_HARDWARE_IMPORT
  #if __GNUC__ >= 4
    #define TAIHU_DUAL_ARM_HARDWARE_PUBLIC __attribute__ ((visibility("default")))
    #define TAIHU_DUAL_ARM_HARDWARE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define TAIHU_DUAL_ARM_HARDWARE_PUBLIC
    #define TAIHU_DUAL_ARM_HARDWARE_LOCAL
  #endif
  #define TAIHU_DUAL_ARM_HARDWARE_PUBLIC_TYPE
#endif

#endif  // TAIHU_DUAL_ARM_HARDWARE__VISIBILITY_CONTROL_H_
