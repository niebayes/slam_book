//@brief Merge the image2 with the image1 using the homography from image2 to
//image1.
#include <vector>

#include "opencv2/opencv.hpp"
#include "utils.h"

int main() {
  cv::Mat image1 = cv::imread("data/3dv_tutorial/images/hill01.jpg");
  cv::Mat image2 = cv::imread("data/3dv_tutorial/images/hill02.jpg");
  ASSERT(image1.data && image2.data && LOAD_ERROR);

  // Retrieve matching points
  cv::Ptr<cv::FeatureDetector> brisk_detector = cv::BRISK::create();
  std::vector<cv::KeyPoint> keypoints1, keypoints2;
  cv::Mat descriptors1, descriptors2;
  brisk_detector->detectAndCompute(image1, cv::Mat{}, keypoints1, descriptors1);
  brisk_detector->detectAndCompute(image2, cv::Mat{}, keypoints2, descriptors2);
  cv::Ptr<cv::DescriptorMatcher> brisk_matcher =
      cv::DescriptorMatcher::create("BruteForce-Hamming");
  std::vector<cv::DMatch> matches;
  brisk_matcher->match(descriptors1, descriptors2, matches);

  // Calculate planar homography and merge them
  std::vector<cv::Point2f> image_points1, image_points2;
  for (auto m = std::cbegin(matches); m < std::cend(matches); ++m) {
    image_points1.emplace_back(keypoints1[m->queryIdx].pt);
    image_points2.emplace_back(keypoints2[m->trainIdx].pt);
  }
  cv::Mat inlier_boolean_mask;  //! This will be updated then by the
                                //! cv::findHomography
  cv::Mat H = cv::findHomography(image_points2, image_points1,
                                 inlier_boolean_mask, cv::RANSAC);
  cv::Mat merged;
  cv::warpPerspective(image2, merged, H,
                      cv::Size(image1.cols * 2, image1.rows));
  merged.colRange(0, image1.cols) = image1 * 1;  //?

  // Show the merged image
  cv::Mat original, matched;
  cv::drawMatches(image1, keypoints1, image2, keypoints2, matches, matched,
                  cv::Scalar::all(-1), cv::Scalar(0, 127, 0),
                  inlier_boolean_mask);
  cv::hconcat(image1, image2, original);
  cv::putText(original, "Original", {5, 25}, cv::FONT_HERSHEY_PLAIN, 2,
              cv::Scalar(0, 255, 0), 2);
  cv::putText(matched, "Matched", {5, 25}, cv::FONT_HERSHEY_PLAIN, 2,
              cv::Scalar(0, 255, 0), 2);
  cv::vconcat(original, matched, matched);
  cv::putText(merged, "Merged", {5, 25}, cv::FONT_HERSHEY_PLAIN, 2,
              cv::Scalar(0, 255, 0), 2);
  cv::vconcat(matched, merged, merged);
  cv::String winname("Image Stitching");
  cv::imshow(winname, merged);
  cv::waitKey(0);
  return EXIT_SUCCESS;
}