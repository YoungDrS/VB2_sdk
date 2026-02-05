#ifndef BRAINCO_REVO1_CONTROL__VISIBILITY_CONTROL_H_
#define BRAINCO_REVO1_CONTROL__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define BRAINCO_REVO1_CONTROL_EXPORT __attribute__((dllexport))
#define BRAINCO_REVO1_CONTROL_IMPORT __attribute__((dllimport))
#else
#define BRAINCO_REVO1_CONTROL_EXPORT __declspec(dllexport)
#define BRAINCO_REVO1_CONTROL_IMPORT __declspec(dllimport)
#endif
#ifdef BRAINCO_REVO1_CONTROL_BUILDING_DLL
#define BRAINCO_REVO1_CONTROL_PUBLIC BRAINCO_REVO1_CONTROL_EXPORT
#else
#define BRAINCO_REVO1_CONTROL_PUBLIC BRAINCO_REVO1_CONTROL_IMPORT
#endif
#define BRAINCO_REVO1_CONTROL_PUBLIC_TYPE BRAINCO_REVO1_CONTROL_PUBLIC
#define BRAINCO_REVO1_CONTROL_LOCAL
#else
#define BRAINCO_REVO1_CONTROL_EXPORT __attribute__((visibility("default")))
#define BRAINCO_REVO1_CONTROL_IMPORT
#if __GNUC__ >= 4
#define BRAINCO_REVO1_CONTROL_PUBLIC __attribute__((visibility("default")))
#define BRAINCO_REVO1_CONTROL_LOCAL __attribute__((visibility("hidden")))
#else
#define BRAINCO_REVO1_CONTROL_PUBLIC
#define BRAINCO_REVO1_CONTROL_LOCAL
#endif
#define BRAINCO_REVO1_CONTROL_PUBLIC_TYPE
#endif

#endif  // BRAINCO_REVO1_CONTROL__VISIBILITY_CONTROL_H_
