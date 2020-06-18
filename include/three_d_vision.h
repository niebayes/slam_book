#ifndef THREE_D_VISION_H_
#define THREE_D_VISION_H_

#include "opencv2/core/core.hpp"

bool is_valid_rotation_mat(cv::Mat &R);

struct EulerAngles {
  double theta_x;  // roll
  double theta_y;  // pitch
  double theta_z;  // yaw
};

EulerAngles decompose_R(cv::Mat &R);

#endif  // THREE_D_VISION_H_