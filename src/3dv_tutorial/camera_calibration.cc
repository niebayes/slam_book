// @cf.
// https://docs.opencv.org/2.4/doc/tutorials/calib3d/camera_calibration/camera_calibration.html

// @cf.
// https://aishack.in/tutorials/calibrating-undistorting-opencv-oh-yeah/

// @cf.
// https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#calibratecamera
#include <fstream>
#include <vector>

#include "opencv2/opencv.hpp"

int main(int argc, char** argv) {
  constexpr auto input = "data/3dv_tutorial/videos/chessboard.avi";
  // * In this case, cv::Size(#inner corners per row, #inner corners per col)
  // * The pattern_size reveals the number of inner corners, excluding the
  // * corners on the edges. It's not dependent on the size of the chessboard
  // * inside the image.
  // ! The pattern_size should be in greatly agree with the target images.
  // ! Any inconsistency or occulusion may cause the cv::findChessboardCorners
  // ! to fail. Hence, you should firstly obtain the size of the pattern cover
  // ! entirely all inner corner.
  const cv::Size pattern_size(10, 7);
  // ! The board_cellsize should also be entirely consistent with the actual
  // ! value. Cause the algorithm searches the corners based on the cell
  // ! interval specified by this variable.
  // * To comply with the Point2f and Point3f, use float type
  // * It's better to append a f or F suffix to the constant literal.
  constexpr float board_cellsize = 0.025f;
  // Alternate this varible to switch between mannual image selection and
  // automatic image acquisition. It depends upon the user.
  constexpr bool select_images = true;

  // Start image acquisition
  // Open a video
  cv::VideoCapture video(input);  // Frame size 480 x 270
  if (!video.isOpened()) return -1;

  // Select images
  std::vector<cv::Mat> images;  // selected images
  cv::Mat original;
  while (true) {
    // Grab an image from the video
    cv::Mat image;
    video >> image;
    // Used for testing calibration later on
    original = image.clone();
    if (image.empty()) break;  // break if reached the end of the video

    if (select_images) {
      // Show the image and keep it if selected
      cv::String winname("Camera Calibration");
      cv::imshow(winname, image);
      cv::String info("Press Enter to acquire an image");
      cv::putText(image, info, cv::Point(20, 40), cv::FONT_HERSHEY_PLAIN, 3,
                  cv::Scalar(0, 255, 0), 3);
      // * while(true) + cv::waitKey(t) resembles the refresh rate t ms.
      int key = cv::waitKey(1);
      if (key == 27) {
        // ! It's better to mannually destory the windows 
        cv::destroyWindow(winname);
        break;               // "ESC" key, exit
      }
      else if (key == 32) {  // "Space" key, pause
        std::vector<cv::Point2f> corners;
        // * The function requires white space (like a square-thick
        // * border, the wider the better) around the board to make the
        // * detection more robust in various environments.
        // * Otherwise, if there is no border and the background is dark, the
        // * outer black squares cannot be segmented properly and so the square
        // * grouping and ordering algorithm fails.
        bool found_and_reordered_all_corners =
            cv::findChessboardCorners(image, pattern_size, corners);
        // * To pause, you need to create a clone, and render it
        // * on top of the previous image (by calling another cv::imshow).
        cv::Mat display = image.clone();
        // * If all corners are found and reordered, draw colored corners
        // * connected with lines, otherwise draw corners as red circles.
        // ! This func and many other OpenCV funcs take as floating type,
        // ! e.g. Point2f, Point3f the default type for "points" and reject
        // ! other types, e.g. Point2d, Point3d
        cv::drawChessboardCorners(display, pattern_size, corners,
                                  found_and_reordered_all_corners);
        cv::imshow(winname, display);
        // * Invoke the pause
        key = cv::waitKey(0);
        if (key == 27) {
          cv::destroyWindow(winname);
          break;
        }
        else if (key == 13) {
          images.push_back(image);  // "Enter" key, select the current image
        }
      }
    } else
      images.push_back(image);  // Invoke in the automatic mode
  }
  video.release();  // End image acquisition
  if (images.empty()) {
    std::cerr << "Failed to grab images\n";
    return -1;
  }

  // Start calibration
  // Find 2D inner corners from acquired images
  // * The outer vector contains the views (images) obtained.
  // * The inner vector contains the array of image points (the inner
  // * corners found by the previous procedures).
  // ! The size of the image_points must be both "outside" and "inside"
  // ! in agree with the object_points, thus the one-to-one correlation can be
  // ! found and used in computing reprojection error later on.
  std::vector<std::vector<cv::Point2f>>
      image_points;  // corners per row and col
  for (std::size_t i = 0; i < images.size(); ++i) {
    std::vector<cv::Point2f> corners;
    if (cv::findChessboardCorners(images[i], pattern_size, corners))
      image_points.push_back(corners);
  }
  if (image_points.empty()) {
    std::cerr << "Failed to find corners!\n";
    return -1;
  }

  // Prepare 3D points of the chessboard
  // * The points are 3D, but since they are in a pattern coordinate system,
  // * then, if the rig is planar (as in this case, the chessboard), it may make
  // * sense to put the model to a XY coordinate plane so that Z-coordinate of
  // * each input object point is 0.
  // ! Here, we use fill constructor in combination with std::vector::resize to
  // ! achieve the best performance, avoding the extra re-allocations if we use
  // ! other methods invoving many move operations.
  std::vector<std::vector<cv::Point3f>> object_points(1);

  // @test
  // cv::Point3f a{1};        // Point3f(1, 0, 0)
  // cv::Point3f c{1, 2, 3};  // Point3f(1, 2, 3)

  // * I guess: when the value passed to the constructor are not valid and
  // * there's no error raised, then the compiler would interpret these
  // * constructor as fill constructors, e.g. std::vector<int> vec(10, 1)
  // * filling in the vec with 10 ones. And even if you use
  // * brace-initialization, aka std::initializer_list, the compiler would
  // * interpret them as the fill constructor.
  // ! Therefore, it's very recommended not using these ambiguious
  // ! constructions. And don't misuse the std::initializer_list as the fill
  // ! constructor. Use parentheses instead, rather than braces in this case.
  // std::vector<cv::Point3f> pts1{1, cv::Point3f{}};
  // std::vector<cv::Point3f> pts2{3, cv::Point3f{}};
  // std::vector<cv::Point3f> pts3{3, };
  // std::vector<cv::Point3f> pts4{3, {}};

  // std::vector<std::vector<cv::Point3f>> object_points1{0};
  // std::vector<std::vector<cv::Point3f>> object_points2{1};
  // @test

  for (int r = 0; r < pattern_size.height; ++r)
    for (int c = 0; c < pattern_size.width; ++c)
      // * Currently, initialization of intrinsic parameters (when
      // * CV_CALIB_USE_INTRINSIC_GUESS is not set) is only implemented for
      // * planar calibration patterns (where Z-coordinates of the object points
      // * must be all zeros, as in this case). Therefore, as we specify the
      // * intrinsic_matrix later on as the initialized identity matrix, we have
      // * to set the Z coordinates of the object_points to zero.
      // * The object_points are just the initialization. They are used to in
      // * combination with the image_points to compute the reprojection error
      // * during the cv::calibrateCamera calling process. Hence, we use the
      // * board_cellsize * #inner_corners to initialize the object_points which
      // * can reduce the reprojection error in the first iteration and make the
      // * convergence faster.
      object_points[0].push_back(
          cv::Point3f(board_cellsize * c, board_cellsize * r,
                      0.0f));  // * It's better to specify the number type
  // * std::vector::resize takes two params: size, value;
  // * The size is the size after the resize operation, and the value
  // * is used when the target size is greater than the original size, to
  // * fill in the remaining entries.
  object_points.resize(image_points.size(), object_points[0]);  // Copy

  // Calibrate the camera
  // ! You can also simply declare the intrinsic_matrix why not initialize it.
  // ! OpenCV will do the dirty things for you.
  cv::Mat intrinsic_matrix = cv::Mat::eye(3, 3, CV_64F);
  // ! It doesn't matter what size you set to the dist_coeffs Mat.
  // ! OpenCV will take care of adapting the size.
  cv::Mat dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);  // k1, k2, p1, p2, p3
  // * OpenCV use the rotation vector and the translation vector to
  // * compactly represent the extrinsic params, aka. the rigid transformation.
  // * The rvecs and tvecs are then computed by cv::Rodrigues() for each
  // * view, aka. each image we thus far obtained.
  std::vector<cv::Mat> rvecs, tvecs;
  // ! The calibrateCamera function converts all matrices into 64F format even
  // ! if you initialize it to 32F.
  double reprojection_error =
      cv::calibrateCamera(object_points, image_points, images[0].size(),
                          intrinsic_matrix, dist_coeffs, rvecs, tvecs);

  // Calibration post test
  cv::Mat calibrated;
  cv::undistort(original, calibrated, intrinsic_matrix, dist_coeffs);
  cv::String win1("Original"), win2("Calibrated");
  cv::imshow(win1, original);
  cv::imshow(win2, calibrated); 
  cv::waitKey(0); // End calibration

  // Report calibration results
  std::ofstream results("data/3dv_tutorial/camera_calibration.txt");
  // cv::FileStorage results("camera_calibration.yaml", cv::FileStorage::WRITE);
  // if (!results.isOpened()) return -1;
  if (!results.is_open()) {
    std::cerr << "Cannot open camera_calibration.txt\n";
    return -1;
  }
  results << "## Camera Calibration Results\n";
  results << "* The number of applied images = " << image_points.size() << '\n';
  results << "* Final reprojection error = " << reprojection_error << '\n';
  results << "* Intrinsic matrix K = \n"
          << "  " << intrinsic_matrix.row(0) << intrinsic_matrix.row(1)
          << intrinsic_matrix.row(2) << '\n';
  results << "* Distortion coefficient (k1, k2, p1, p2, k3, ...) = \n"
          << "  " << dist_coeffs.t() << '\n';
  results.close();
  std::cerr << "  Saved the results in camera_calibration.txt\n";
  return 0;
}