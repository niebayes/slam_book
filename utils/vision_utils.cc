#include "vision_utils.h"

#include <cassert>
#include <cmath>

#include "opencv2/core/core.hpp"

bool is_valid_R(cv::Mat &R) {
  // @brief check if the mat R is a valid rotation matrix
  // by the property: R * R^T or R^T * R = I
  cv::Mat Rt = R.t();
  cv::Mat res = R * Rt;
  cv::Mat I = cv::Mat::eye(R.rows, R.cols, res.type());
  // return the absolute difference norm, default L-2 norm
  return cv::norm(res, I) < 1e-6;
}

EulerAngles decompose_R(cv::Mat &R) {
  // * cf. http://nghiaho.com/?page_id=846
  // @ brief decompose the valid rotation matrix to the set of euler angles
  // which are bound in range.
  // @return theta_x: [-pi, pi]
  // @return theta_y: [-pi/2, pi/2]
  // @return theta_z: [-pi, pi]

  // ! Never return references of pointers to local variables
  EulerAngles euler;
  assert(is_valid_R(R));
  euler.theta_x = std::atan2(R.at<double>(3, 2), R.at<double>(3, 3));
  euler.theta_y = std::atan2(-R.at<double>(3, 1),
                             std::sqrt(std::pow(R.at<double>(3, 2), 2)) +
                                 std::pow(R.at<double>(3, 3), 2));
  euler.theta_z = std::atan2(R.at<double>(2, 1), R.at<double>(1, 1));
  return euler;
}

// * Macro alternatives.
// #define Rx(rx)      (cv::Matx33d(1, 0, 0, 0, cos(rx), -sin(rx), 0, sin(rx),
// cos(rx))) #define Ry(ry)      (cv::Matx33d(cos(ry), 0, sin(ry), 0, 1, 0,
// -sin(ry), 0, cos(ry))) #define Rz(rz)      (cv::Matx33d(cos(rz), -sin(rz), 0,
// sin(rz), cos(rz), 0, 0, 0, 1))
cv::Matx33d euler_angles_to_rotation_matrix(cv::Point3d &p) {
  // * When combined with << operator or with a user-defined data type for
  // * entries, use cv::Mat_
  // * Otherwise, use cv::Mat and specify the size and the type in the
  // * constructor arguments.
  cv::Matx33d Rx(1, 0, 0, 0, std::cos(p.x), -std::sin(p.x), 0, std::sin(p.x),
                 std::cos(p.x));
  cv::Matx33d Ry(std::cos(p.y), 0, std::sin(p.y), 0, 1, 0, -std::sin(p.y), 0,
                 std::cos(p.y));
  cv::Matx33d Rz(std::cos(p.z), -std::sin(p.z), 0, std::sin(p.z), std::cos(p.z),
                 0, 0, 0, 1);
  cv::Matx33d R = Rz * Ry * Rx;
  // * "If you need to do some operation on Matx that is not implemented, it is
  // * easy to convert the matrix to Mat and backwards."
  cv::Mat R_(R);
  // ! assert is a macro, hence the argument should be known at compile time.
  assert(is_valid_R(R_));
  return R;
}