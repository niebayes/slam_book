#include "vision_book_utils.h"

#include <algorithm>  // std::max, std::min

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

//! non-integral type 'double' is an invalid underlying type.
//! Hence, we need to wrap the constants into a struct/class.
struct Weights {
  //@cf. The color weights
  // https://docs.opencv.org/4.3.0/de/d25/imgproc_color_conversions.html

  // Follow the OpenCV convention BGR
  //! To initialize outside the struct/class (without constexpr qualifier),
  //! use the scope resolution operator ::, otherwise use "dot" operator.
  static constexpr double B = 0.114;
  static constexpr double G = 0.587;
  static constexpr double R = 0.299;
};

//@cf. at() template func
// https://docs.opencv.org/master/d3/d63/classcv_1_1Mat.html#aa5d20fc86d41d59e4d71ae93daee9726
cv::Mat rgb_to_gray(cv::Mat &src, Weights &w) {
  ASSERT(src.channels() == 3 && "Invalid image");
  //* Mat.type() returns int, which'll be interpreted as enum, e.g. CV_8UC1
  //! It's highlt recommended that using cv::Mat_ to explicitly specify the
  //! data type. Hence the compiling time checking works. (??)
  cv::Mat_<uchar> gray = cv::Mat::zeros(src.size(), CV_8UC1);

  //@cf. cv::Mat element access methods
  // https://docs.opencv.org/4.3.0/db/da5/tutorial_how_to_scan_images.html
  //* Three methods: at (safest), pointer (fastest), iterator (neat and safe)
  cv::MatIterator_<uchar> gray_it(&gray);
  for (auto it = src.begin<cv::Vec3b>(); it < src.end<cv::Vec3b>(); ++it) {
    auto intensity = (*it)[0] * w.B + (*it)[1] * w.G + (*it)[2] * w.R;
    // intensity = intensity > 255 ? 255 : intensity;
    // intensity = intensity < 0 ? 0 : intensity;
    //* A neater way to bound the value: max(min()) couple.
    //! To use std::max or std::min, the type of the x, y must be the same.
    *gray_it++ =
        std::max((uchar)0, std::min((uchar)255, static_cast<uchar>(intensity)));
  }

  return gray;
}

int main() {
  constexpr auto file("data/3dv_book/images/lena.png");

  // OpenCV API
  //* cv::IMREAD_COLOR if set, always convert image to the 3 channel
  //* BGR color image.
  cv::Mat src = cv::imread(file, cv::IMREAD_COLOR);
  cv::Mat gray_opencv;
  //* In case of non-linear transformations, you'd better scale the src image
  //* to the dst's data range, e.g. src /= 255 or src *= 1./255 in the case of
  //* transforming CV_8U* image to CV_32F* image. Otherwise, you'd lose info.
  //* It's recommended using CV_32F* when doing image transformations.
  cv::cvtColor(src, gray_opencv, cv::COLOR_BGR2GRAY, 1);
  cv::imshow("Gray OpenCV", gray_opencv);
  cv::waitKey(0);

  // Impl
  Weights w;
  auto gray = rgb_to_gray(src, w);
  cv::imshow("Gray Impl", gray);
  cv::waitKey(0);
  return 0;
}

//@cf. Image pyramid in OpenCV
// https://docs.opencv.org/4.3.0/d4/d1f/tutorial_pyramids.html

//@cf. Non-maximum suppression (NMS)
// https://towardsdatascience.com/non-maximum-suppression-nms-93ce178e177c

//@cf. Image load and save, self implemantation
// https://github.com/opencv/opencv/blob/master/modules/imgcodecs/src/loadsave.cpp