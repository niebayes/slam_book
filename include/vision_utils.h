#ifndef VISION_UTILS_H_
#define VISION_UTILS_H_

#include "opencv2/core/core.hpp"

bool is_valid_R(cv::Mat &R);

struct EulerAngles {
  double theta_x;  // roll
  double theta_y;  // pitch
  double theta_z;  // yaw
};

EulerAngles decompose_R(cv::Mat &R);

cv::Matx33d euler_angles_to_rotation_matrix(cv::Point3d& p);

#endif  // VISION_UTILS_H_