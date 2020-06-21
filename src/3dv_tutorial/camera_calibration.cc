#include <fstream>
#include <vector>

#include "opencv2/opencv.hpp"

int main(int argc, char** argv) {
  constexpr auto input = "data/3dv_tutorial/videos/chessboard.avi";
  // ! In this case, cv::Size(#inner corners per row, #inner corners per col)
  cv::Size board_pattern(10, 7);
  double board_cellsize = 0.025;
  bool select_images = true;

  // Open a video
  cv::VideoCapture video(input);
  if (!video.isOpened()) return -1;

  // Select images
  std::vector<cv::Mat> image_c;  // image container
  while (true) {
    // Grab an image from the video
    cv::Mat image;
    video >> image;
    if (image.empty()) break;  // break if reached the end of the video

    if (select_images) {
      // Show the image and keep it if selected
      cv::String winname("Camera Calibration");
      cv::imshow(winname, image);
      // * while(true) + cv::waitKey(t) resembles the refresh rate t ms.
      int key = cv::waitKey(1);
      if (key == 27)
        break;              // "ESC" key, exit
      else if (key == 32) {  // "Space" key, pause
        std::vector<cv::Point2d> point_c;
        bool found_and_reordered_all_corners =
            cv::findChessboardCorners(image, board_pattern, point_c);
        cv::Mat display = image.clone();
        cv::drawChessboardCorners(display, board_pattern, point_c,
                                  found_and_reordered_all_corners);
        cv::imshow(winname, display);
        key = cv::waitKey(0);
        if (key == 27)
          break;
        else if (key == 13)
          image_c.push_back(image);  // "Enter" key, select the current image
      }
    } else
      image_c.push_back(image);
  }
  video.release();
  if (image_c.empty()) {
    std::cerr << "Failed to grab images\n";
    return -1;
  }

  // Find 2D inner corners in the given images
  std::vector<std::vector<cv::Point2d>>
      inner_corners;  // corners per row and col
  for (std::size_t i = 0; i < image_c.size(); ++i) {
    std::vector<cv::Point2d> point_c;
    if (cv::findChessboardCorners(image_c[i], board_pattern, point_c))
      inner_corners.push_back(point_c);
  }
  if (inner_corners.empty()) return -1;

  // Prepare 3D points of the chessboard
  std::vector<std::vector<cv::Point3d>> object_points{1}; // ?
  for (int r = 0; r < board_pattern.height; ++r)
    for (int c = 0; c < board_pattern.width; ++c)
      object_points[0].push_back(
          cv::Point3d(board_cellsize * c, board_cellsize * r, 0));
  object_points.resize(inner_corners.size(), object_points[0]);  // Copy

  // Calibrate the camera
  cv::Mat P = cv::Mat::eye(3, 3, CV_64F);
  cv::Mat dist_coeff = cv::Mat::zeros(4, 1, CV_64F);
  std::vector<cv::Mat> rvecs, tvecs;
  double rms_error =
      cv::calibrateCamera(object_points, inner_corners, image_c[0].size(), P,
                          dist_coeff, rvecs, tvecs);

  // Report calibration results
  std::ofstream result("data/3dv_tutorial/camera_calibration.txt");
  // cv::FileStorage result("camera_calibration.yaml", cv::FileStorage::WRITE);
  // if (!result.isOpened()) return -1;
  if (!result.is_open()) return -1;
  result << "## Camera Calibration Results\n";
  result << "* The number of applied images = " << inner_corners.size()
         << std::endl;
  result << "* RMS error = " << rms_error << std::endl;
  result << "* Camera matrix P = " << std::endl
         << "  " << P.row(0) << P.row(1) << P.row(2) << std::endl;
  result << "* Distortion coefficient (k1, k2, p1, p2, k3, ...) = " << std::endl
         << "  " << dist_coeff.t() << std::endl;
  result.close();
  return 0;
}