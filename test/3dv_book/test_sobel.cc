#include <iostream>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "operators.h"
#include "vision_book_utils.h"

void test_sobel() {
  //@opencv Sobel
  //@cf.
  // https://docs.opencv.org/4.3.0/d2/d2c/tutorial_sobel_derivatives.html
  const cv::String file("data/3dv_book/images/bikesgray.jpg");
  cv::Mat src = cv::imread(file, cv::IMREAD_COLOR);
  const cv::String winname("Output");
  ASSERT(src.data && "Failed to load image!");
  cv::Mat src_gray;
  cv::cvtColor(src, src_gray, cv::COLOR_BGR2GRAY, 1);
  cv::Mat grad_x, grad_y;
  cv::Mat abs_grad_x, abs_grad_y;
  int dst_depth = CV_32F;

  // cv::Sobel(src_gray, grad_x, dst_depth, 1, 0);
  // cv::Sobel(src_gray, grad_y, dst_depth, 0, 1);

  // cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, output);
  // cv::imshow(winname, output);
  // cv::waitKey(0);

  //@impl Sobel
  impl::SobelOperator sobel_op;
  // sobel_op.Sobel(src_gray, grad_x, sobel_op.X, dst_depth,
  // sobel_op.ZERO_BORDER); sobel_op.Sobel(src_gray, grad_y, sobel_op.Y,
  // dst_depth, sobel_op.ZERO_BORDER);

  cv::convertScaleAbs(grad_x, abs_grad_x);
  cv::convertScaleAbs(grad_y, abs_grad_y);
  cv::Mat output;
  sobel_op.Sobel(src_gray, output, sobel_op.XY, dst_depth,
                 sobel_op.ZERO_BORDER);

  cv::imshow(winname, output);
  cv::waitKey(0);
}

int main() { test_sobel(); }