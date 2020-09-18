#ifdef WIN32
#   ifdef Moticon_EXPORTS
#       define Moticon_API __declspec(dllexport)
#   else
#       define Moticon_API  __declspec(dllimport)
#   endif
#else
#   define Moticon_API
#endif // WIN32
