#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define MscController_DLLIMPORT __declspec(dllimport)
#  define MscController_DLLEXPORT __declspec(dllexport)
#  define MscController_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define MscController_DLLIMPORT __attribute__((visibility("default")))
#    define MscController_DLLEXPORT __attribute__((visibility("default")))
#    define MscController_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define MscController_DLLIMPORT
#    define MscController_DLLEXPORT
#    define MscController_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef MscController_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define MscController_DLLAPI
#  define MscController_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef MscController_EXPORTS
#    define MscController_DLLAPI MscController_DLLEXPORT
#  else
#    define MscController_DLLAPI MscController_DLLIMPORT
#  endif // MscController_EXPORTS
#  define MscController_LOCAL MscController_DLLLOCAL
#endif // MscController_STATIC