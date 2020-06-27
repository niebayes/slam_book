#ifndef UTILS_KERNELS_H_
#define UTILS_KERNELS_H_

#include "opencv2/core/core.hpp"

// TODO What's the correct way to define kernel macros?
#define AVERAGE_KERNEL (1. / 9 * cv::Mat{1, 1, 1, 1, 1, 1, 1, 1, 1})
#define GAUSSIAN_KERNEL (1. / 16 * cv::Mat{1, 2, 1, 2, 4, 2, 1, 2, 1})
#define SHARPEN_KERNEL \
  cv::Mat_<char> { 0, -1, 0, -1, 5, -1, 0, -1, 0 }

#define SOBEL_AVG_KERNEL \
  cv::Mat_<char> { 1, 2, 1 }
#define SOBEL_DIFF_KERNEL \
  cv::Mat_<char> { 1, 0, -1 }

#define SCHARR_AVG_KERNEL \
  cv::Mat_<char> { 3, 10, 3 }
#define SCHARR_DIFF_KERNEL \
  cv::Mat_<char> { 1, 0, 1 }
#endif  // UTILS_KERNELS_H_