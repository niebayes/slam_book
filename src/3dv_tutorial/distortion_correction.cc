// * OpenCV video input tutorial, cf.
// * https://docs.opencv.org/4.3.0/d5/dc4/tutorial_video_input_psnr_ssim.html

// * OpenCV Flags for video I/O, cf.
// * https://docs.opencv.org/4.3.0/d4/d15/group__videoio__flags__base.html#ggaeb8dd9c89c10a5c63c139bf7c4f5704da6223452891755166a4fd5173ea257068

// * cv::remap cf.
// * https://docs.opencv.org/3.4/d1/da0/tutorial_remap.html

// * cv::initUndistortRectifyMap cf.
// * https://docs.opencv.org/3.1.0/da/d54/group__imgproc__transform.html#ga7dfb72c9cf9780a347fbe3d1c47e5d5a
#include <vector>

#include "opencv2/opencv.hpp"

int main(int argc, char** argv) {
  constexpr auto input = "data/3dv_tutorial/video/chessboard.avi";
  // ! In OpenCV, camera matrix = intrinsic matrix, and the extrinsic
  // ! matrix is specified with the rotation vector and the translation vector.
  const cv::Matx33d K{432.7390364738057,
                      0,
                      476.0614994349778,
                      0,
                      431.2395555913084,
                      288.7602152621297,
                      0,
                      0,
                      1};
  // Given camera distortion coefficients
  const std::vector<double> dist_coeffs{
      -0.2852754904152874, 0.1016466459919075, -0.0004420196146339175,
      0.0001149909868437517, -0.01803978785585194};

  // Open a video
  cv::VideoCapture video(input);
  if (!video.isOpened()) return -1;
  // ! d and i are signed int type specifiers, while f floating point (double)
  // ! type specifier
  std::cout << cv::format("Read a video with frames' size %.0f x %.0f\n",
                          video.get(cv::CAP_PROP_FRAME_WIDTH),
                          video.get(cv::CAP_PROP_FRAME_HEIGHT));

  // Run distortion correction
  bool show_rectify = true;
  cv::Mat map1, map2;
  while (true) {
    // Read an image from the video
    cv::Mat image;
    // * >> operator = cv::VideoCapture::read = cv::VideoCapture::grab +
    // * cv::VideoCapture::retrieve
    video >> image;
    if (image.empty()) break;
    // TODO
    // bool map_acquired = false;
    // if (!map_acquired)

    // Rectify geometric distortion
    cv::String info("Original");
    if (show_rectify) {
      if (map1.empty() || map2.empty()) {
        // * OpenCV internal cv::InputArray can be constructed from cv::Mat,
        // * std::vector, std::vector<std::vector>, and many others.
        // * cv::undistort() = cv::initUndistortRectifyMap() with unity R +
        // * cv::remap() with bilinear interpolation
        // * When distoring many images, e.g. images read from a video,
        // * it's better to use cv::initUndistortRectifyMap once and followed by
        // * various times of cv::remap to accelerate distortion.
        // ! In monocular case, the param R is assumed to be unity R, you can
        // ! only also pass an empty cv::Mat(), OpenCV will implicitly replace
        // ! it; In stereo case, the rotation matrix R computed by
        // ! cv::stereoRectify() can be passed here.
        cv::initUndistortRectifyMap(K, dist_coeffs, cv::Mat(), cv::Mat(),
                                    image.size(), CV_32FC1, map1, map2);
      }
      cv::remap(image, image, map1, map2, cv::InterpolationFlags::INTER_LINEAR);
      info = "Rectified";
    }
    cv::putText(image, info, cv::Point(5, 15), cv::FONT_HERSHEY_PLAIN, 1,
                cv::Scalar(0, 255, 0));

    // Show the image
    constexpr auto winname = "Distortion Correction";
    cv::imshow(winname, image);
    //! Consider using char key = static_cast<char>(cv::waitKey(1)) in
    //! combination with e.g. if (key = 'c') to make the statement more
    //! expressive.
    int key = cv::waitKey(1);
    if (key == 27)
      break;  // "ESC" key, exit the program
    else if (key == 9)
      show_rectify ^= 1;   // "Tab" key, toggle show_rectify
    else if (key == 32) {  // "Space" key, pause
      key = cv::waitKey(0);
      if (key == 27)
        break;
      else if (key == 9)
        show_rectify ^= 1;
    }
  }
  // * Close video file or capturing device
  video.release();
  return 0;
}