#include <vector>

#include "opencv2/opencv.hpp"
#include "utils.h"

int main() {
  // Open a video and get the reference image and feature points
  cv::String file("data/3dv_tutorial/videos/traffic.avi");
  cv::VideoCapture video(file);
  ASSERT(video.isOpened() && LOAD_ERROR);
  cv::Mat gray_ref;
  video >> gray_ref;
  ASSERT(gray_ref.data && LOAD_ERROR);
  if (gray_ref.channels() > 1)
    cv::cvtColor(gray_ref, gray_ref,
                 cv::COLOR_BGR2GRAY);  //! cv::goodFeaturesToTrack need a single
                                       //! channel image.
  std::vector<cv::Point2f> points_ref;
  cv::goodFeaturesToTrack(
      gray_ref, points_ref, 2000, 0.01,
      10);  //* Find the most prominent corners in the image.
  ASSERT(points_ref.size() >= 4 && INSUFFICIENT_ERROR);

  // Run and show video stabilization
  while (true) {
    cv::Mat frame, frame_gray;
    video >> frame;
    if (frame.empty()) break;
    if (frame.channels() > 1)
      cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);
    else
      frame_gray = frame.clone();

    // Extract optical flow and calculate planar homography
    std::vector<cv::Point2f> points;
    std::vector<uchar> status;  //* 1 if the corresponding features has been
                                //* found, 0 otherwise.
    cv::Mat errors,
        inlier_mask;  //! To create vectors, use std::vector or cv::Mat
    cv::calcOpticalFlowPyrLK(
        gray_ref, frame_gray, points_ref, points, status,
        errors);  //* Utilize optical flow to track the features (calculate the
                  //* new positions of the points)
    cv::Mat H = cv::findHomography(
        points, points_ref, inlier_mask,
        cv::RANSAC);  //* Find homography from original to target.

    // Synthesize a stabilized image
    cv::Mat warped;
    cv::warpPerspective(frame, warped, H, frame.size());

    // Show the original and rectified images together
    for (std::size_t i = 0; i < points_ref.size(); ++i) {
      if (inlier_mask.at<uchar>(i))
        cv::line(frame, points_ref[i], points[i], cv::Scalar(0, 0, 255));
      else
        cv::line(frame, points_ref[i], points[i], cv::Scalar(0, 127, 0));
    }  //! Draw lines connecting the matched points.
       //! This is done in a single image to show the result of the optical flow
       //! tracking.
    cv::putText(frame, "Original", {5, 30}, cv::FONT_HERSHEY_PLAIN, 2,
                cv::Scalar(0, 255, 0), 2);
    cv::putText(warped, "Stabilized", {5, 30}, cv::FONT_HERSHEY_PLAIN, 2,
                cv::Scalar(0, 255, 0), 2);
    cv::hconcat(frame, warped, frame);
    cv::String winname("Video Stabilization");
    cv::imshow(winname, frame);
    char key = cv::waitKey(1);
    if (key == 27)
      break;  // 'ESC' key -> exit
    else if (key == 32)
      cv::waitKey(0);  // 'Space' key -> pause
  }
  video.release();
  return EXIT_SUCCESS;
}