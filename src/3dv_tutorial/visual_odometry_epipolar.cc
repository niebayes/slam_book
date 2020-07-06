#include <fstream>
#include <vector>

#include "opencv2/opencv.hpp"
#include "utils.h"

int main() {
  cv::String file("data/3dv_tutorial/KITTI_07_L/%06d.png");

  // Given settings
  double f = 707.0912;
  cv::Point2d c(601.8873, 183.1104);  // Principal point
  bool use_5pts = true;               // five points method
  int min_inlier_num = 100;

  // Open a file to write camera trajectory
  std::ofstream camera_traj("visual_odometry_epipolar.xyz", std::ios::out);
  ASSERT(camera_traj && LOAD_ERROR);

  // Open a video and get the initial image
  cv::VideoCapture video(file);
  ASSERT(video.isOpened() && LOAD_ERROR);
  cv::Mat prev_gray;
  video >> prev_gray;
  ASSERT(prev_gray.data && LOAD_ERROR);
  if (prev_gray.channels() > 1)
    cv::cvtColor(prev_gray, prev_gray, cv::COLOR_BGR2GRAY);

  // Run and record monocular visual odometry
  cv::Mat camera_pose = cv::Mat::eye(4, 4, CV_64FC1);
  while (true) {
    cv::Mat frame, gray;
    video >> frame;
    if (frame.empty()) break;
    if (frame.channels() > 1)
      cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    else
      gray = frame.clone();

    // Extract optical flow
    std::vector<cv::Point2f> prev_points, points;
    cv::goodFeaturesToTrack(prev_gray, prev_points, 2000, 0.01, 10);
    std::vector<uchar> status;
    cv::Mat errors;
    cv::calcOpticalFlowPyrLK(prev_gray, gray, prev_points, points, status,
                             errors);
    prev_gray = gray;

    // Calculate relative pose
    cv::Mat E, inlier_mask;
    if (use_5pts) {
      E = cv::findEssentialMat(prev_points, points, f, c, cv::RANSAC, 0.99, 1.0,
                               inlier_mask);
    } else {
      cv::Mat F = cv::findFundamentalMat(prev_points, points, cv::FM_RANSAC,
                                         1.0, 0.99, inlier_mask);
      cv::Matx33d K{f,   0, c.x, 0, f,
                    c.y, 0, 0,   1};  //! Cannot construct cv::Mat in this way.
                                      //! Use cv::Matx33d instead.
      E = K.t() * F * K;              //?
    }
    cv::Mat R, t;
    int n_inlier = cv::recoverPose(
        E, prev_points, points, R, t, f, c,
        inlier_mask);  //! This function decomposes an essential matrix using
                       //! @ref decomposeEssentialMat and then verifies possible
                       //! pose hypotheses by doing cheirality check. The
                       //! cheirality check means that the triangulated 3D
                       //! points should have positive depth. Returns the number
                       //! of inliers that pass the check.
    // Accumulate relative pose if result is reliable
    if (n_inlier >= min_inlier_num) {
      cv::Mat T = cv::Mat::eye(4, 4, R.type());
      T(cv::Rect(0, 0, 3, 3)) = R * 1.0;  //? copy
      T.col(3).rowRange(0, 3) = t * 1.0;
      camera_pose = camera_pose * T.inv();
    }

    // Show the image and write camera pose
    if (frame.channels() < 3) cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);
    for (std::size_t i = 0; i < prev_points.size(); ++i) {
      if (inlier_mask.at<uchar>(i))
        cv::line(frame, prev_points[i], points[i], cv::Scalar(0, 0, 255));
      else
        cv::line(frame, prev_points[i], points[i], cv::Scalar(0, 127, 0));
    }

    cv::String info(cv::format(
        "Inliers: %i (%lu%%), XYZ: [%.3f, %.3f, %.3f]", n_inlier,
        100 * n_inlier / points.size(), camera_pose.at<double>(0, 3),
        camera_pose.at<double>(1, 3),
        camera_pose.at<double>(
            2, 3)));  //* Present relative camera position (translation).
    cv::putText(frame, info, {5, 30}, cv::FONT_HERSHEY_PLAIN, 2,
                cv::Scalar(0, 255, 0));
    cv::String winname("Visual Odometry -- Epipolar");
    cv::imshow(winname, frame);
    camera_traj << cv::format("%.3f, %.3f, %.3f", camera_pose.at<double>(0, 3),
                              camera_pose.at<double>(1, 3),
                              camera_pose.at<double>(2, 3));
    char key = cv::waitKey(1);
    if (key == 27)
      break;  // 'ESC' key -> exit
    else if (key == 32)
      cv::waitKey(0);  // 'Space' key -> pause
  }
  video.release();
  camera_traj.close();
  return EXIT_SUCCESS;
}