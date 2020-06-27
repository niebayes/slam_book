#ifndef BORDERS_H_
#define BORDERS_H_

#include "opencv2/core/core.hpp"

namespace impl {
void add_zero_border(cv::Mat &src) {
  src.row(0).setTo(cv::Scalar(0));
  src.row(src.rows - 1).setTo(cv::Scalar(0));
  src.col(0).setTo(cv::Scalar(0));
  src.col(src.cols - 1).setTo(cv::Scalar(0));
}
}  // namespace impl

#endif  // BORDERS_H_