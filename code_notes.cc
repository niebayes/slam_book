#include <algorithm>  // std::copy
#include <opencv2/opencv.hpp>
#include <vector>

int main() {
  // Simply convert std::vector<Point2d> to std::vector<Point>
  std::vector<cv::Point2d> vd{{1.1, 2.2}, {3.3, 4.4}, {5.5, 6.6}};
  std::vector<cv::Point> v(vd.begin(), vd.end());

  // Print for debug
  // ! Awesome STL-style cout
  std::copy(vd.begin(), vd.end(),
            std::ostream_iterator<cv::Point2d>(std::cout, " "));
  std::cout << std::endl;
  std::copy(v.begin(), v.end(),
            std::ostream_iterator<cv::Point>(std::cout, " "));

  return 0;
}