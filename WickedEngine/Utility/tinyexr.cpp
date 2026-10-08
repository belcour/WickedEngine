// tinyexr implementation (C++, so it cannot live in the extern "C" block of utility_common.cpp)
// ZIP decompression uses the stb_image / stb_image_write zlib compiled in utility_common.cpp
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif // _CRT_SECURE_NO_WARNINGS

#define TINYEXR_USE_MINIZ 0
#define TINYEXR_USE_STB_ZLIB 1
#define TINYEXR_USE_THREAD 1
#define TINYEXR_USE_OPENMP 0
#define TINYEXR_IMPLEMENTATION
#include "tinyexr.h"
