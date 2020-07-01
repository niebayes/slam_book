#include <vector>

#include "const_iter.h"
#include "macros_util.h"
#include "opencv2/opencv.hpp"

int main() {
  // Given settings
  const cv::String file("data/3dv_tutorial/videos/blais.mp4"),
      cover("data/3dv_tutorial/images/blais.jpg");
  double f = 1000, cx = 320, cy = 240;
  cv::Matx33d K{f, 0, cx, 0, f, cy, 0, 0, 1};
  cv::Mat dist_coeffs = cv::Mat::zeros(5, 1, CV_64FC1);
  cv::Mat rvec, tvec;
  std::size_t min_inliner_num = 100;

  // Prepare a 3D box for simple AR
  std::vector<cv::Point3f> box_lower{
      {30, 145, 0}, {200, 145, 0}, {200, 200, 0}, {30, 200, 0}};
  std::vector<cv::Point3f> box_upper{
      {30, 145, -50}, {200, 145, -50}, {200, 200, -50}, {30, 200, -50}};

  // Load the object image
  cv::Mat object_image = cv::imread(cover);
  ASSERT(object_image.data && LOAD_ERROR);

  // Extract the ORB features in the cover image
  cv::Ptr<cv::FeatureDetector> orb_detector = cv::ORB::create();
  std::vector<cv::KeyPoint> object_keypoints;
  cv::Mat object_descriptor;  //! Descriptor is a vector. In OpenCV, vectors are
                              //! created generally with cv::Mat
  orb_detector->detectAndCompute(object_image, cv::Mat{}, object_keypoints,
                                 object_descriptor);
  ASSERT(object_keypoints.data() && object_descriptor.data && COMPUTE_ERROR);

  // Prepare a train set for matching descriptors
  auto orb_matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
  orb_matcher->add(object_descriptor);  // Add the train set

  // Start matching
  cv::VideoCapture video(file);
  ASSERT(video.isOpened() && LOAD_ERROR);

  // Run pose estimation
  while (true) {
    // Grab an image from the video
    cv::Mat image;
    video >> image;
    ASSERT(image.data && LOAD_ERROR);

    // Extract ORB features in this frame
    std::vector<cv::KeyPoint> image_keypoints;
    cv::Mat image_descriptors;
    orb_detector->detectAndCompute(image, cv::Mat{}, image_keypoints,
                                   image_descriptors);
    if (image_keypoints.empty() || image_descriptors.empty())
      continue;  // Continue if the feature extraction failed

    // Match the descriptors of this frame with the train set (the descriptors
    // of the cover image)
    std::vector<cv::DMatch>
        matches;  // Stores the matched index pair and the score
    orb_matcher->match(image_descriptors, matches);
    if (matches.size() < min_inliner_num)
      continue;  // Continue if the #matched pair < threshold

    std::vector<cv::Point3f> object_points;
    // std::vector<cv::Point2f> object_projects, image_points; //?
    std::vector<cv::Point2f> image_points;
    // Construct "matches1to2"
    for (auto m = std::cbegin(matches); m < std::cend(matches); ++m) {
      object_points.emplace_back(object_keypoints[m->trainIdx].pt);
      // object_projects.emplace_back(object_keypoints[m->trainIdx].pt);
      image_points.emplace_back(image_keypoints[m->queryIdx].pt);
    }

    // Reject outliers, the wrongly matched feature pair.
    std::vector<char> inliers;  // Store the indices of the inliers
    cv::solvePnPRansac(object_points, image_points, K, dist_coeffs, rvec, tvec,
                       false, 500, 2.0F, 0.99, inliers);
    cv::Mat inlier_mask = cv::Mat::zeros(
        static_cast<int>(matches.size()), 1,
        CV_8UC1);  // Boolean mask, not to be confused with the image mask
    for (std::size_t i = 0; i < inliers.size(); ++i)
      inlier_mask.at<uchar>(inliers[i]) = 1;  // If inlier, set 1

    cv::Mat image_result;
    cv::drawMatches(image, image_keypoints, object_image, object_keypoints,
                    matches, image_result, cv::Scalar(0, 0, 255),
                    cv::Scalar(0, 127, 0), inlier_mask);

    // Estimate camera pose with inliers
    std::size_t inlier_num = inliers.size();
    if (inlier_num > min_inliner_num) {
      std::vector<cv::Point3f> object_inliers;
      std::vector<cv::Point2f> image_inliers;
      for (int idx = 0; idx < inlier_mask.rows; ++idx) {
        if (inlier_mask.at<uchar>(idx)) { //?
          object_inliers.emplace_back(object_points[idx]);
          image_inliers.emplace_back(image_points[idx]);
        }
      }
      // Update the camera pose: rvec and tvec
      cv::solvePnP(object_points, image_points, K, dist_coeffs, rvec, tvec);

      // Draw the box on the image, using the rectified camera pose
      cv::Mat line_lower, line_upper;
      cv::projectPoints(box_lower, rvec, tvec, K, dist_coeffs, line_lower);
      cv::projectPoints(box_upper, rvec, tvec, K, dist_coeffs, line_upper);
      line_lower.reshape(1).convertTo(line_lower, CV_32S);
      line_upper.reshape(1).convertTo(line_upper, CV_32S);
      cv::polylines(image_result, line_lower, true, cv::Scalar(255, 0, 0), 2);
      cv::polylines(image_result, line_upper, true, cv::Scalar(0, 0, 255), 2);
      for (int i = 0; i < line_lower.rows; ++i)
        cv::line(image_result, {line_lower.row(i)}, {line_upper.row(i)},
                 cv::Scalar(0, 255, 0), 2);
    }
    // Show the image
    cv::String info(cv::format("Inliers: %lu (%lu%%), Focal Length: %.0f",
                               inlier_num, 100 * inlier_num / matches.size(),
                               K(0, 0)));
    cv::putText(image_result, info, cv::Point(5, 30), cv::FONT_HERSHEY_PLAIN, 2,
                cv::Scalar(0, 255, 0));
    const cv::String winname("Pose Estimation -- Book 1");
    cv::imshow(winname, image_result);
    char key = cv::waitKey(1);
    if (key == 27) break;
    else if (key == 32) cv::waitKey(0);
  }
  video.release();
  return EXIT_SUCCESS;
}