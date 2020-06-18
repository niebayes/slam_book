#include "three_d_vision.h"

#include <cassert>
#include <cmath>

#include "opencv2/core/core.hpp"

bool is_valid_rotation_mat(cv::Mat &R) {
  // check if the mat R is a valid rotation matrix
  // by the property: R * R^T or R^T * R = I
  cv::Mat Rt = R.t();
  cv::Mat res = R * Rt;
  cv::Mat I = cv::Mat::eye(R.rows, R.cols, res.type());
  // return the absolute difference norm, default L-2 norm
  return cv::norm(res, I) < 1e-6;
}

EulerAngles decompose_R(cv::Mat &R) {
  // * cf. http://nghiaho.com/?page_id=846
  // decompose the valid rotation matrix to the set of euler angles
  // which are bound in range.
  // @return theta_x: [-pi, pi]
  // @return theta_y: [-pi/2, pi/2]
  // @return theta_z: [-pi, pi]

  // ! Never return references of pointers to local variables
  EulerAngles euler;
  assert(is_valid_rotation_mat(R));
  euler.theta_x = std::atan2(R.at<double>(3, 2), R.at<double>(3, 3));
  euler.theta_y = std::atan2(-R.at<double>(3, 1),
                             std::sqrt(std::pow(R.at<double>(3, 2), 2)) +
                                 std::pow(R.at<double>(3, 3), 2));
  euler.theta_z = std::atan2(R.at<double>(2, 1), R.at<double>(1, 1));
  return euler;
}