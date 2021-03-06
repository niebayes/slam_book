#include <cmath>
#include <fstream>
#include <iomanip>  // std::boolalpha
#include <iostream>
#include <string>
#include <typeinfo>
#include <vector>

#include "opencv2/opencv.hpp"
#include "pcd_io_utils.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "vision_utils.h"

// Rotation around major axes
// * OpenCV is row-major which stores elements row by row
// * function-like define macros are helpful in certain cases
// #define Rx(rx)                                                           \
//   (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, std::cos(rx), -std::sin(rx), 0, \
//    std::sin(rx), std::cos(rx))
// #define Ry(ry)                                                       \
//   (cv::Mat_<double>(3, 3) << std::cos(ry), 0, std::sin(ry), 0, 1, 0, \
//    -std::sin(ry), 0, std::cos(ry))
// #define Rz(rz)                                                             \
//   (cv::Mat_<double>(3, 3) << std::cos(rz), -std::sin(rz), 0, std::sin(rz), \
//    std::cos(rz), 0, 0, 0, 1)

// * Alternative to macros: a function
// * Access entries in Vec: using indices;
// * Access entries in Point: using x, y, z, ... members.
cv::Mat euler_angles_to_rotation_matrix(cv::Point3d& p) {
  // * When combined with << operator or with a user-defined data type for
  // * entries, use cv::Mat_
  // * Otherwise, use cv::Mat and specify the size and the type in the
  // * constructor arguments.
  cv::Mat Rx = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, std::cos(p.x),
                -std::sin(p.x), 0, std::sin(p.x), std::cos(p.x));
  cv::Mat Ry = (cv::Mat_<double>(3, 3) << std::cos(p.y), 0, std::sin(p.y), 0, 1,
                0, -std::sin(p.y), 0, std::cos(p.y));
  cv::Mat Rz = (cv::Mat_<double>(3, 3) << std::cos(p.z), -std::sin(p.z), 0,
                std::sin(p.z), std::cos(p.z), 0, 0, 0, 1);
  cv::Mat R = Rz * Ry * Rx;
  return R;
}

int main(int argc, char** argv) {
  // Given camera parameters: focal length, principal point (to denote the
  // position of the image plane), image resolution (width x height), camera
  // position, and camera orientation
  // * The unit is pixel.
  double f = 1000, ox = 320, oy = 240, noise_sigma = 1;
  // * cv::Size, template class for specifying the size of an image or
  // * rectangle.
  cv::Size img_res(640, 480);  // * image resolution, width x height
  using point = cv::Point3d;
  // * T object(), direct initialization (construction);
  // * T object{...}, list initialization, usually used with vector-like
  // * objects.
  std::vector<point> cam_pos{point(0, 0, 0), point(-2, -2, 0), point(2, 2, 0),
                             point(-2, 2, 0), point(2, -2, 0)};
  // Specify the orientation of the camera with Euler angles
  // * type alias declaration: using identifier = type-id
  // * CV_PI is a macro name that refers to a constant, not a type
  // * Hence alias declaration is not applicable on CV_PI
  // * Instead, we should use auto to emulate the alias declaration
  auto pi = CV_PI;
  std::vector<point> cam_ori{
      point(0, 0, 0), point(-pi / 12, pi / 12, 0), point(pi / 12, -pi / 12, 0),
      point(pi / 12, pi / 12, 0), point(-pi / 12, -pi / 12, 0)};

  // * fstream.open() can be constructed with c-style string or string in C++
  // * So it's okay not using c_str().
  // * However, when combined with argv, which usually is a char*,
  // * it's better to use c_str().
  // * cf. https://stackoverflow.com/q/6396330/11240780
  std::ifstream fin("data/3dv_tutorial/box.xyz", std::ios::in);
  if (!fin.is_open()) return -1;
  // ! cv::Mat is just an n-dimensional dense array
  // ! When the X.push_bacl operations over, X becomes an array in which
  // ! elements are cv::Vec4d.
  // ! Hence, we need to reshape X to a rectangular-like matrix for the matrix
  // ! operations later on.
  cv::Mat X;
  std::string data;
  while (std::getline(fin, data)) {
    double x, y, z;
    std::istringstream istr(data);
    // ? The >> operator automatically do type conversion and casting?
    istr >> x >> y >> z;
    // Homogeneous coord.
    // * Mat.push_back(), add a line (row) to the bottom of the Mat
    // * Emulate the action of the push_back() of std::vector.
    X.push_back(cv::Vec4d(x, y, z, 1));
  }
  fin.close();
  // Convert to a 4 x N matrix
  X = X.reshape(1).t();

  // Generate images for each camera pose
  cv::Mat K{(cv::Mat_<double>(3, 3) << f, 0, ox, 0, f, oy, 0, 0, 1)};
  for (std::size_t i = 0; i < cam_pos.size(); ++i) {
    // Derive a projection matrix
    // * We choose this order of rotations combination
    // * because it corresponds the yaw, pitch, roll convention
    cv::Mat R = euler_angles_to_rotation_matrix(cam_ori[i]);
    cv::Mat T(cam_pos[i]);
    cv::Mat g;
    cv::hconcat(R, T, g);
    cv::Mat P = K * g;
    // cv::Mat T(cam_pos[i]);
    // cv::Mat g;
    // ? Why inverse the rigid transformation matrix
    // cv::hconcat(R.t(), -R.t() * T, g);
    // cv::Mat P = K * g;

    // Project the points
    cv::Mat x = P * X;
    x.row(0) = x.row(0) / x.row(2);
    x.row(1) = x.row(1) / x.row(2);
    x.row(2) = 1;

    // Add Gaussian noise
    cv::Mat noise(2, x.cols, x.type());
    cv::randn(noise, cv::Scalar(0), cv::Scalar(noise_sigma));
    x.rowRange(0, 2) += noise;

    // Show and store the points
    cv::Mat image = cv::Mat::zeros(img_res, CV_8UC1);
    for (int c = 0; c < x.cols; ++c) {
      cv::Point p(x.col(c).rowRange(0, 2));
      if (p.x >= 0 && p.x < img_res.width && p.y >= 0 && p.y < img_res.height)
        cv::circle(image, p, 2, 255, -1);
    }
    // * cv::format acts like printf
    // * In C++ 20, we can use std::format defined in the <format> header
    // * Otherwise, we can use boost::format with the boost lib installed.
    cv::imshow(cv::format("Image Formation %lu", i), image);

    // Store the projected points in pcd file
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.width = x.cols;
    cloud.height = 1;
    for (std::size_t c = 0; c < x.cols; ++c) {
      // cv::Point p(x.col(c).rowRange(0, 2));
      // cloud.emplace_back(p.x, p.y, 1);
      cloud.emplace_back(x.at<double>(0, c), x.at<double>(1, c), 1);
    }
    pcl::io::savePCDFileASCII(
        cv::format("data/3dv_tutorial/pcd/image_formation%lu.pcd", i), cloud);

    std::ofstream fout(
        cv::format("data/3dv_tutorial/out/image_formation%lu.xyz", i),
        std::ios::out);

    if (!fout.is_open()) return -1;
    for (int c = 0; c < x.cols; ++c)
      fout << cv::format("%f %f 1\n", x.at<double>(0, c), x.at<double>(1, c));
    fout.close();
  }

  cv::waitKey(0);
  return 0;
}