#ifndef OPERATORS_H_
#define OPERATORS_H_

#include <iostream>
#include <vector>

#include "borders.h"
#include "kernels.h"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "vision_book_utils.h"

namespace impl {

class SobelOperator {
 public:
  enum Gradient { X, Y, XY };

  enum BorderTypes { CONSTANT_BORDER, REPLICATED_BORDER, ZERO_BORDER };

  void Sobel(const cv::Mat &src, cv::Mat &dst, int gradient, int dst_depth,
             int border_type) {
    ASSERT(src.data && "Empty source image");
    ASSERT(src.channels() == 1 && "Takes as input only gray images");

    cv::Mat kernel;
    cv::Mat grad_x, grad_y, magnitude;
    switch (gradient) {
      case Gradient::X:
        kernel = SOBEL_GX_KERNEL;
        convolve(src, grad_x, kernel, dst_depth, border_type);
        dst = grad_x;
        std::cerr << "Convolved with the Horizontal Sobel kernel\n";
        break;

      case Gradient::Y:
        kernel = SOBEL_GY_KERNEL;
        convolve(src, grad_y, kernel, dst_depth, border_type);
        dst = grad_y;
        std::cerr << "Convolved with the Vertical Sobel kernel\n";
        break;

      case Gradient::XY:
        Sobel(src, grad_x, Gradient::X, dst_depth, border_type);
        Sobel(src, grad_y, Gradient::Y, dst_depth, border_type);
        cv::magnitude(grad_x, grad_y, magnitude);
        dst = magnitude;
        std::cerr
            << "Convolved with the Horizontal and Vertical Sobel kernels\n";
        break;

      default:
        std::cerr << "Invalid gradient type!\n";
        return;
        break;
    }
  }

  static void convolve(const cv::Mat &src, cv::Mat &dst, const cv::Mat &kernel,
                       int dst_depth, int border_type) {
    //@cf. Simple and intuitive implementation for convolution
    // https://www.programming-techniques.com/2013/02/calculating-convolution-of-image-with-c_2.html

    ASSERT(src.data && "Failed to load image");
    ASSERT(src.depth() == CV_8U && "Takes as input only uchar images");
    ASSERT(kernel.size() == cv::Size(3, 3) && "Kernel size must be 3 x 3")
    ASSERT(border_type == BorderTypes::ZERO_BORDER &&
           "Only support ZERO_BORDER type");

    const int n_channels = src.channels();
    dst.create(src.size(), dst_depth);

    for (std::size_t r = 1; r < src.rows - 1; ++r) {
      const auto previous_row = src.ptr<uchar>(r - 1);
      const auto current_row = src.ptr<uchar>(r);
      const auto next_row = src.ptr<uchar>(r + 1);

      auto dst_ptr = dst.ptr<float>(r);
      for (std::size_t c = n_channels; c < n_channels * (src.cols - 1); ++c) {
        std::size_t left_col = c - n_channels;
        std::size_t curr_col = c;
        std::size_t right_col = c + n_channels;
        std::vector<char> weights(kernel.begin<char>(), kernel.end<char>());
        auto intensity = weights[8] * previous_row[left_col] +
                         weights[7] * previous_row[curr_col] +
                         weights[6] * previous_row[right_col] +
                         weights[5] * current_row[left_col] +
                         weights[4] * current_row[curr_col] +
                         weights[3] * current_row[right_col] +
                         weights[2] * next_row[left_col] +
                         weights[1] * next_row[curr_col] +
                         weights[0] * next_row[right_col];
        *dst_ptr++ = cv::saturate_cast<float>(intensity);
      }
    }
    if (border_type == ZERO_BORDER) impl::add_zero_border(dst);
  }

 private:
  cv::Mat SOBEL_GX_KERNEL =
      (cv::Mat_<char>(3, 3) << 1, 0, -1, 2, 0, -2, 1, 0, -1);
  cv::Mat SOBEL_GY_KERNEL =
      (cv::Mat_<char>(3, 3) << 1, 2, 1, 0, 0, 0, -1, -2, -1);
};

}  // namespace impl

#endif  // OPERATORS_H_