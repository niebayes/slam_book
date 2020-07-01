#include <vector>

#include "const_iter.h"
#include "macros_util.h"
#include "opencv2/opencv.hpp"

int main() {
  const cv::String file("data/3dv_tutorial/videos/blais.mp4"),
      cover("data/3dv_tutorial/images/blais.jpg");

  // Given settings
  double f_init = 1000, cx_init = 320, cy_init = 240;
  std::size_t min_inlier_num = 100;

  // Prepare a box for simple AR
  std::vector<cv::Point3f> box_lower = {
      {30, 145, 0}, {30, 200, 0}, {200, 200, 0}, {200, 145, 0}};
  std::vector<cv::Point3f> box_upper = {
      {30, 145, -50}, {30, 200, -50}, {200, 200, -50}, {200, 145, -50}};

  // Load the object image and extract features
  cv::Mat object_image = cv::imread(cover);
  ASSERT(object_image.data && LOAD_ERROR);

  auto orb_detector = cv::ORB::create();
  auto orb_matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
  std::vector<cv::KeyPoint> object_keypoints;
  cv::Mat object_descriptors;
  orb_detector->detectAndCompute(object_image, cv::Mat{}, object_keypoints,
                                 object_descriptors);
  ASSERT(object_keypoints.data() && object_descriptors.data && COMPUTE_ERROR);
  orb_matcher->add(object_descriptors);

  // Open a video
  cv::VideoCapture video(file);
  ASSERT(video.isOpened() && LOAD_ERROR);

  // Run pose estimation and camera calibration together
  cv::Matx33d K{f_init, 0, cx_init, 0, f_init, cy_init, 0, 0, 1};
  cv::Mat dist_coeffs = cv::Mat::zeros(5, 1, CV_64FC1);
  cv::Mat rvec, tvec;

  while (true) {
    // Grab an image from the video
    cv::Mat frame;
    video >> frame;
    ASSERT(frame.data && LOAD_ERROR);

    // Extract features and match them to the object features
    std::vector<cv::KeyPoint> image_keypoints;
    cv::Mat image_descriptors;
    orb_detector->detectAndCompute(frame, cv::Mat{}, image_keypoints,
                                   image_descriptors);
    if (image_keypoints.empty() || image_descriptors.empty()) continue;
    std::vector<cv::DMatch> matches;
    orb_matcher->match(image_descriptors, matches);
    if (matches.size() < min_inlier_num) continue;
    std::vector<cv::Point3f> object_points;
    std::vector<cv::Point2f> image_points;
    for (auto m = std::cbegin(matches); m < std::cend(matches); ++m) {
      object_points.emplace_back(object_keypoints[m->trainIdx].pt);
      image_points.emplace_back(image_keypoints[m->queryIdx].pt);
    }

    // Reject outliers
    std::vector<int> inliers;
    cv::solvePnPRansac(object_points, image_points, K, dist_coeffs, rvec, tvec,
                       false, 500, 2.0F, 0.99, inliers);
    cv::Mat inlier_boolean_mask =
        cv::Mat::zeros(static_cast<int>(matches.size()), 1, CV_8UC1);
    for (std::size_t i = 0; i < inliers.size(); ++i)
      inlier_boolean_mask.at<uchar>(inliers[i]) = 1;

    cv::Mat image_result;
    cv::drawMatches(frame, image_keypoints, object_image, object_keypoints,
                    matches, image_result, cv::Scalar(0, 0, 255),
                    cv::Scalar(0, 127, 0), inlier_boolean_mask);

    // Calibrate the camera and estimate its pose with inliers
    std::size_t n_inlier = inliers.size();
    if (n_inlier > min_inlier_num) {
      std::vector<cv::Point3f> object_inliers;
      std::vector<cv::Point2f> image_inliers;
      for (int i = 0; i < inlier_boolean_mask.rows; ++i) {
        if (inlier_boolean_mask.at<uchar>(i)) {
          object_inliers.emplace_back(object_points[i]);
          image_inliers.emplace_back(image_points[i]);
        }
      }
      std::vector<cv::Mat> rvecs, tvecs;
      cv::calibrateCamera(
          std::vector<std::vector<cv::Point3f>>(1, object_inliers),
          std::vector<std::vector<cv::Point2f>>(1, image_inliers), frame.size(),
          K, dist_coeffs, rvecs, tvecs,
          cv::CALIB_FIX_ASPECT_RATIO | cv::CALIB_FIX_PRINCIPAL_POINT |
              cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_FIX_K1 |
              cv::CALIB_FIX_K2 | cv::CALIB_FIX_K3 | cv::CALIB_FIX_K4 |
              cv::CALIB_FIX_K5 | cv::CALIB_FIX_K6 | cv::CALIB_FIX_S1_S2_S3_S4 |
              cv::CALIB_FIX_TAUX_TAUY);
      rvec = rvecs[0].clone();
      tvec = tvecs[0].clone();

      // Draw the box on the image
      cv::Mat line_lower, line_upper;
      cv::projectPoints(box_lower, rvec, tvec, K, dist_coeffs, line_lower);
      cv::projectPoints(box_upper, rvec, tvec, K, dist_coeffs, line_upper);
      line_lower.reshape(1).convertTo(
          line_lower,
          CV_32S);  // Change 4 x 1 matrix (CV_64FC2) to 4 x 2 matrix (CV_32SC1)
      line_upper.reshape(1).convertTo(
          line_upper,
          CV_32S);  //  because 'cv::polylines()' only accepts 'CV_32S' depth.
      cv::polylines(image_result, line_lower, true, cv::Scalar(255, 0, 0), 2);
      cv::polylines(image_result, line_upper, true, cv::Scalar(0, 0, 255), 2);
      for (int i = 0; i < line_lower.rows; i++)
        cv::line(image_result, cv::Point(line_lower.row(i)),
                 cv::Point(line_upper.row(i)), cv::Scalar(0, 255, 0), 2);
    }

    // Show the image
    cv::String info(cv::format("Inliers: %lu (%lu%%), Focal Length: %.0f",
                               n_inlier, 100 * n_inlier / matches.size(),
                               K(0, 0)));
    cv::putText(image_result, info, cv::Point(5, 30), cv::FONT_HERSHEY_PLAIN, 2,
                cv::Scalar(0, 255, 0));
    cv::imshow("Pose Estimation -- Book 2", image_result);
    char key = cv::waitKey(1);
    if (key == 27)
      break;  // 'ESC' key: Exit
    else if (key == 32)
      cv::waitKey(0);
  }
  video.release();
  return EXIT_SUCCESS;
}