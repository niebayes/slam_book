#include <ostream>
#include <vector>

#include "const_iter.h"
#include "macros_util.h"
#include "opencv2/opencv.hpp"

int main() {
  const cv::String file("data/3dv_tutorial/videos/blais.mp4"),
      cover("data/3dv_tutorial/images/blais.jpg");

  // Given settings
  std::size_t min_inlier_num = 100;

  // Prepare a box for simple AR
  std::vector<cv::Point3f> box_lower = {
      {30, 145, 0}, {30, 200, 0}, {200, 200, 0}, {200, 145, 0}};
  std::vector<cv::Point3f> box_upper = {
      {30, 145, -50}, {30, 200, -50}, {200, 200, -50}, {200, 145, -50}};

  // Train
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

  // Query
  cv::VideoCapture video(file);
  ASSERT(video.isOpened() && LOAD_ERROR);
  // Run pose estimation and camera calibration together
  while (true) {
    // Match and extract the matched points
    cv::Mat image;
    video >> image;
    if (!image.data) break;
    std::vector<cv::KeyPoint> image_keypoints;
    cv::Mat image_descriptors;
    orb_detector->detectAndCompute(image, cv::Mat{}, image_keypoints,
                                   image_descriptors);
    if (image_keypoints.empty() || image_descriptors.empty()) continue;
    std::vector<cv::DMatch> matches;
    orb_matcher->match(image_descriptors, matches);
    if (matches.size() < min_inlier_num)
      continue;  //! At this stage, the "inliers" are actually all the matches
                 //! before rejecting outliers.
    std::vector<cv::Point3f> object_points;
    std::vector<cv::Point2f> object_projects, image_points;
    for (auto m = std::cbegin(matches); m < std::cend(matches); ++m) {
      object_points.emplace_back(object_keypoints[m->trainIdx].pt);
      object_projects.emplace_back(object_keypoints[m->trainIdx].pt);
      image_points.emplace_back(image_keypoints[m->queryIdx].pt);
    }

    // Reject outliers
    cv::Mat inlier_boolean_mask = cv::Mat::zeros(
        static_cast<int>(matches.size()), 1,
        CV_8UC1);  //! Inliers are wrt. the result of finding Homography
    cv::Mat H = cv::findHomography(
        image_points, object_projects, inlier_boolean_mask, cv::RANSAC,
        2);  //! The reprojThreshold is usually set in range [1, 10];
    cv::Mat image_result;
    cv::drawMatches(image, image_keypoints, object_image, object_keypoints,
                    matches, image_result, cv::Scalar(0, 0, 255),
                    cv::Scalar(0, 127, 0), inlier_boolean_mask);

    // Calibrate the camera and estimate its pose with inliers
    // Construct the inlier sets
    double f;  //! For presentation
    std::size_t n_inlier =
        static_cast<std::size_t>(cv::sum(inlier_boolean_mask)[0]);
    if (n_inlier > min_inlier_num) {
      std::vector<cv::Point3f> object_inliers;
      std::vector<cv::Point2f> image_inliers;
      for (int i = 0; i < inlier_boolean_mask.rows; ++i) {
        if (inlier_boolean_mask.at<uchar>(i)) {
          object_inliers.emplace_back(object_points[i]);
          image_inliers.emplace_back(image_points[i]);
        }
      }

      // Calibrate the camera
      cv::Mat K, dist_coeffs;
      std::vector<cv::Mat> rvecs, tvecs;
      cv::calibrateCamera(
          std::vector<std::vector<cv::Point3f>>(1, object_inliers),
          std::vector<std::vector<cv::Point2f>>(1, image_inliers), image.size(),
          K, dist_coeffs, rvecs, tvecs,
          cv::CALIB_FIX_ASPECT_RATIO | cv::CALIB_FIX_PRINCIPAL_POINT |
              cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_FIX_K1 |
              cv::CALIB_FIX_K2 | cv::CALIB_FIX_K3 | cv::CALIB_FIX_K4 |
              cv::CALIB_FIX_K5 | cv::CALIB_FIX_K6 | cv::CALIB_FIX_S1_S2_S3_S4 |
              cv::CALIB_FIX_TAUX_TAUY);
      cv::Mat rvec = rvecs[0].clone();
      cv::Mat tvec = tvecs[0].clone();
      f = K.at<double>(0);

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
                               n_inlier, 100 * n_inlier / matches.size(), f));
    cv::putText(image_result, info, cv::Point(5, 30), cv::FONT_HERSHEY_PLAIN, 2,
                cv::Scalar(0, 255, 0));
    cv::imshow("Pose Estimation -- Book 3", image_result);
    char key = cv::waitKey(1);
    if (key == 27)  // 'ESC' key -> exit
      break;
    else if (key == 32)  // 'space' key -> pause
      cv::waitKey(0);
  }
  video.release();
  return EXIT_SUCCESS;
}