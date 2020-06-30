#ifndef MACROS_UTIL_H_
#define MACROS_UTIL_H_

#include <cstdlib>   // std::abort
#include <iostream>  // std::cerr

#include "opencv2/core/core.hpp"  // cv::format

//* Use if-else in place of the ?: conditional ternary operator
//* to avoid strict type alignment.
#ifndef NDEBUG
#define FMT                                                            \
  "  Assertion Failed @\n    FILE:       %s\n    LINE:       %d\n    " \
  "FUNCTION:   %s\n"
#define ASSERT(expression)                                                \
  if (!(expression)) {                                                    \
    std::cerr << cv::format(FMT, __FILE__, __LINE__, __PRETTY_FUNCTION__) \
              << "    EXPRESSION: " << #expression << '\n';               \
    std::abort();                                                         \
  }
#else
#define ASSERT(expression) ((void)0)
#endif

//@cf. CV_Assert
// https://docs.opencv.org/master/db/de0/group__core__utils.html#gaf62bcd90f70e275191ab95136d85906b
//* The CV_Assert defined by OpenCV
// #define CV_Assert(expr)                                                    \
//   do {                                                                     \
//     if (!!(expr))                                                          \
//       ;                                                                    \
//     else                                                                   \
//       cv::error(cv::Error::StsAssert, #expr, CV_Func, __FILE__, __LINE__); \
//   } while (0)

#endif  // MACROS_UTIL_H_