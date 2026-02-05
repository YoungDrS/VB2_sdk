#ifndef JAKA_DUAL_CONTROL__VISIBILITY_CONTROL_H_
#define JAKA_DUAL_CONTROL__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define JAKA_DUAL_CONTROL_EXPORT __attribute__((dllexport))
#define JAKA_DUAL_CONTROL_IMPORT __attribute__((dllimport))
#else
#define JAKA_DUAL_CONTROL_EXPORT __declspec(dllexport)
#define JAKA_DUAL_CONTROL_IMPORT __declspec(dllimport)
#endif
#ifdef JAKA_K1_DUAL_CONTROL_BUILDING_DLL
#define JAKA_DUAL_CONTROL_PUBLIC JAKA_DUAL_CONTROL_EXPORT
#else
#define JAKA_DUAL_CONTROL_PUBLIC JAKA_DUAL_CONTROL_IMPORT
#endif
#define JAKA_DUAL_CONTROL_PUBLIC_TYPE JAKA_DUAL_CONTROL_PUBLIC
#define JAKA_DUAL_CONTROL_LOCAL
#else
#define JAKA_DUAL_CONTROL_EXPORT __attribute__((visibility("default")))
#define JAKA_DUAL_CONTROL_IMPORT
#if __GNUC__ >= 4
#define JAKA_DUAL_CONTROL_PUBLIC __attribute__((visibility("default")))
#define JAKA_DUAL_CONTROL_LOCAL __attribute__((visibility("hidden")))
#else
#define JAKA_DUAL_CONTROL_PUBLIC
#define JAKA_DUAL_CONTROL_LOCAL
#endif
#define JAKA_DUAL_CONTROL_PUBLIC_TYPE
#endif

#endif  // JAKA_DUAL_CONTROL__VISIBILITY_CONTROL_H_
