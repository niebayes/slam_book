#include <vector>

#include "macros_util.h"
#include "opencv2/opencv.hpp"

int main() {
  const cv::String file("data/3dv_tutorial/videos/chessboard.avi");
  cv::Matx33d K{432.7390364738057,
                0,
                476.0614994349778,
                0,
                431.2395555913084,
                288.7602152621297,
                0,
                0,
                1};
  std::vector<double> dist_coeffs{-0.2852754904152874, 0.1016466459919075,
                                  -0.0004420196146339175, 0.0001149909868437517,
                                  -0.01803978785585194};
  cv::Size pattern_size(10, 7);  // Induce the 9 x 6 inner pattern.
  //? How this value impact the final computed camera position?
  //? What else does this value impacts?
  double board_cellsize = 0.025;  // Unit meter

  // Open a video
  cv::VideoCapture video(file);
  ASSERT(video.isOpened() && "Unable to open the video");

  // Prepare a 3D box for simple AR.
  std::vector<cv::Point3d> box_lower{
      {4 * board_cellsize, 2 * board_cellsize, 0},
      {5 * board_cellsize, 2 * board_cellsize, 0},
      {5 * board_cellsize, 4 * board_cellsize, 0},
      {4 * board_cellsize, 4 * board_cellsize, 0},
  };

  std::vector<cv::Point3d> box_upper{
      {4 * board_cellsize, 2 * board_cellsize, -board_cellsize},
      {5 * board_cellsize, 2 * board_cellsize, -board_cellsize},
      {5 * board_cellsize, 4 * board_cellsize, -board_cellsize},
      {4 * board_cellsize, 4 * board_cellsize, -board_cellsize},
  };

  // Prepare 3D points on a chessboard
  std::vector<cv::Point3d> object_points;
  for (int r = 0; r < pattern_size.height; ++r)
    for (int c = 0; c < pattern_size.width; ++c)
      object_points.push_back({board_cellsize * c, board_cellsize * r, 0});

  // Run pose estimation
  while (true) {
    // Grab an image from the video
    cv::Mat image;
    video >> image;
    ASSERT(image.data && "Unable to grab the image");

    // Estimate the camera pose
    std::vector<cv::Point2d> image_points;
    bool has_found =
        cv::findChessboardCorners(image, pattern_size, image_points);
    if (has_found) {
      cv::Mat rvec, tvec;
      cv::solvePnP(object_points, image_points, K, dist_coeffs, rvec, tvec);

      // Draw the box on this image
      cv::Mat line_lower, line_upper;
      cv::projectPoints(box_lower, rvec, tvec, K, dist_coeffs, line_lower);
      cv::projectPoints(box_upper, rvec, tvec, K, dist_coeffs, line_upper);
      // std::cerr << line_lower.channels() << "\n"; // #channels = 2
      //@note cv::Mat::reshape cf.
      // https://docs.opencv.org/trunk/d3/d63/classcv_1_1Mat.html#a4eb96e3251417fa88b78e2abd6cfd7d8
      //@note cv::Mat::convertTo cf.
      // https://docs.opencv.org/2.4/modules/core/doc/basic_structures.html
      line_lower.reshape(1).convertTo(  // cv::polylines takes points as input
          line_lower,  // Hence, we need to change the #channels.
          CV_32S);     // Because 'cv::polylines()' only accepts 'CV_32S' depth.
      line_upper.reshape(1).convertTo(line_upper, CV_32S);
      // Draw polygonal lines (折线) connecting these points contained in
      // line_lower / line_upper
      cv::polylines(image, line_lower, true, cv::Scalar(255, 0, 0),
                    2);  // BGR color order
      cv::polylines(image, line_upper, true, cv::Scalar(0, 0, 255), 2);
      // Connect the points between that in line_lower and that in line_upper
      for (int i = 0; i < line_lower.rows; ++i)
        cv::line(image, cv::Point(line_lower.row(i)),
                 cv::Point(line_upper.row(i)), cv::Scalar(0, 255, 0), 2);

      // Print camera pose (actually, only the position (translation wrt. world
      // coord.))
      cv::Mat R;
      cv::Rodrigues(rvec, R);
      cv::Mat pose = -R.t() * tvec;
      cv::Point3d XYZ(pose);
      cv::String info =
          cv::format("XYZ: [%.3f, %.3f, %.3f]", XYZ.x, XYZ.y, XYZ.z);
      //! cv::Point(x_along_width, y_along_height)
      cv::putText(image, info, cv::Point(15, 30), cv::FONT_HERSHEY_PLAIN, 2,
                  cv::Scalar(0, 255, 0));
    }

    // Show this image
    const cv::String winname("Pose Estimation -- Chessboard");
    cv::imshow(winname, image);
    char key = cv::waitKey(1);
    if (key == 27) break;  // 'ESC' key -> exit
  }
  video.release();
  return 0;
}
