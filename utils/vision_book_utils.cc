#include "vision_book_utils.h"

#include <algorithm>  // std::max, std::min

#include "kernels.h"
#include "opencv2/core/core.hpp"            // Basic Mat operators
#include "opencv2/highgui/highgui.hpp"      // cv::imshow, cv::waitKey
#include "opencv2/imgcodecs/imgcodecs.hpp"  // cv::imread, cv::imwrite
#include "opencv2/imgproc/imgproc.hpp"  // Corner, edge detector; cv::filter2D

//! non-integral type 'double' is an invalid underlying type.
//! Hence, we need to wrap the constants into a struct/class.
struct Weights {
  //@cf. The color weights
  // https://docs.opencv.org/4.3.0/de/d25/imgproc_color_conversions.html

  // Follow the OpenCV convention BGR
  //! To initialize outside the struct/class (without constexpr qualifier),
  //! use the scope resolution operator ::, otherwise use "dot" operator.
 public:
  static constexpr double B = 0.114;
  static constexpr double G = 0.587;
  static constexpr double R = 0.299;
};

//@cf. at() template func
// https://docs.opencv.org/master/d3/d63/classcv_1_1Mat.html#aa5d20fc86d41d59e4d71ae93daee9726
//! Be sure to add const qualifier if you won't change them.
cv::Mat rgb_to_gray(const cv::Mat &src, const Weights &w) {
  ASSERT(src.channels() == 3 && "Invalid image");
  //* Mat.type() returns int, which'll be interpreted as enum, e.g. CV_8UC1
  //! It's highlt recommended that using cv::Mat_ to explicitly specify the
  //! data type. Hence the compiling time checking works. (??)
  cv::Mat_<uchar> gray = cv::Mat::zeros(src.size(), CV_8UC1);

  //@cf. cv::Mat element access methods
  // https://docs.opencv.org/4.3.0/db/da5/tutorial_how_to_scan_images.html
  //* Three methods: at (safest), pointer (fastest), iterator (neat, safe but
  //* slower)
  cv::MatIterator_<uchar> gray_it(&gray);
  for (auto it = src.begin<cv::Vec3b>(); it < src.end<cv::Vec3b>(); ++it) {
    // TODO
    // Use cv::Matx31d to store the weights, and do matrix multiplication
    // with the cv::Vec3b (maybe transformed into cv::Mat) to get a neater
    // expression.
    auto intensity = (*it)[0] * w.B + (*it)[1] * w.G + (*it)[2] * w.R;
    // intensity = intensity > 255 ? 255 : intensity;
    // intensity = intensity < 0 ? 0 : intensity;
    //* A neater way to bound the value: max(min()) couple.
    //! To use std::max or std::min, the type of the x, y must be the same.
    //@cf. OpenCV built-in cv::saturate_cast
    // https://docs.opencv.org/4.3.0/db/de0/group__core__utils.html#gab93126370b85fda2c8bfaf8c811faeaf
    // uchar a = cv::saturate_cast<uchar>(-100);   // a = 0   (uchar_MIN)
    // uchar b = cv::saturate_cast<uchar>(300);    // b = 255 (uchar_MAX)
    // uchar c = cv::saturate_cast<uchar>(185.55); // c = 186 (Rounded)
    // std::cerr << (int)a << " " << (int)b << " " << (int)c << '\n';
    //! As the examples above, the max_min couple can be replaced by a single
    //! cv::saturate_cast<uchar>() casting operation.
    *gray_it++ =
        std::max((uchar)0, std::min((uchar)255, static_cast<uchar>(intensity)));
  }

  return gray;
}

//! It's recommended not returing a cv::Mat instance but passing a reference to
//! dst image, to coincide with the OpenCV convention.
void sharpen(const cv::Mat &src, cv::Mat &dst) {
  //@cf. Mask operations on matrices in OpenCV
  // https://docs.opencv.org/4.3.0/d7/d37/tutorial_mat_mask_operations.html

  // Make sure the data type consistent with our codes below.
  //* Mat::depth vs. Mat::type
  ASSERT(src.depth() == CV_8U && "Takes as input only uchar images");
  const int n_channels = src.channels();
  dst.create(src.size(), src.type());

  for (std::size_t r = 1; r < src.rows - 1; ++r) {
    const auto previous_row = src.ptr<uchar>(r - 1);
    const auto current_row = src.ptr<uchar>(r);
    const auto next_row = src.ptr<uchar>(r + 1);

    auto dst_ptr = dst.ptr<uchar>(r);
    for (std::size_t c = n_channels; c < n_channels * (src.cols - 1); ++c) {
      auto intensity = 5 * current_row[c] - current_row[c - n_channels] -
                       current_row[c + n_channels] - previous_row[c] -
                       next_row[c];
      *dst_ptr++ = cv::saturate_cast<uchar>(intensity);
    }
  }

  //! Considier pad the src image first to handle missing pixels along
  //! the boundaries.
  //@cf. cv::copyMakeBorder(), BORDER_CONSTANT, BORDER_REPLICATE
  // https://docs.opencv.org/3.4/dc/da3/tutorial_copyMakeBorder.html
  dst.row(0).setTo(cv::Scalar(0));
  dst.row(dst.rows - 1).setTo(cv::Scalar(0));
  dst.col(0).setTo(cv::Scalar(0));
  dst.col(dst.cols - 1).setTo(cv::Scalar(0));
}

int main() {
  constexpr auto file("data/3dv_book/images/lena.png");

  //* cv::IMREAD_COLOR if set, always convert image to the 3 channel
  //* BGR color image.
  cv::Mat src = cv::imread(file, cv::IMREAD_COLOR);
  cv::imshow("Source Image", src);
  cv::waitKey(0);

  //@opencv cv::cvtColor
  cv::Mat gray_opencv;
  //* In case of non-linear transformations, you'd better scale the src image
  //* to the dst's data range, e.g. src /= 255 or src *= 1./255 in the case of
  //* transforming CV_8U* image to CV_32F* image. Otherwise, you'd lose info.
  //* It's recommended using CV_32F* when doing image transformations.
  cv::cvtColor(src, gray_opencv, cv::COLOR_BGR2GRAY, 1);
  cv::imshow("Gray OpenCV", gray_opencv);
  cv::waitKey(0);

  //@impl
  Weights w;
  auto gray = rgb_to_gray(src, w);
  cv::imshow("Gray Impl", gray);
  cv::waitKey(0);

  //@opencv cv::filter2D
  // TODO Predefine some commonly used macros in kernels.h
  cv::Mat kernel = (cv::Mat_<char>(3, 3) << 0, -1, 0, -1, 5, -1, 0, -1, 0);
  cv::Mat sharpened2;
  cv::filter2D(src, sharpened2, src.depth(), kernel);
  cv::imshow("Shapened Image OpenCV", sharpened2);
  cv::waitKey(0);

  //@impl
  cv::Mat sharpened;
  sharpen(src, sharpened);
  cv::imshow("Shapened Image", sharpened);
  cv::waitKey(0);

  //@ cf. overloaded operator =, defined between cv::Mat and cv::Scalar.
  /** @brief Sets all or some of the array elements to the specified value.
  @param s Assigned scalar converted to the actual array type.
  */
  // gray_opencv = cv::Scalar(0);  // Set all elements to zero (black)
  // cv::Scalar::all(0);
  // cv::imshow("Gray OpenCV", gray_opencv);
  // cv::waitKey(0);

  return 0;
}

//@cf. Image pyramid in OpenCV
// https://docs.opencv.org/4.3.0/d4/d1f/tutorial_pyramids.html

//@cf. Non-maximum suppression (NMS)
// https://towardsdatascience.com/non-maximum-suppression-nms-93ce178e177c

//@cf. Image load and save, self implemantation
// https://github.com/opencv/opencv/blob/master/modules/imgcodecs/src/loadsave.cpp