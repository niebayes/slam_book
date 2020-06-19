#include <string>

#include "opencv2/opencv.hpp"
#include "vision_utils.h"

// @brief cast degrees to radius
// * Alternative: use macro.
// * Pros: dynamic-type argument; Cons: coincide with other macros.
// #define DEG2RAD(v) ((v) * CV_PI / 180)
double deg2rad(double v) { return v * CV_PI / 180; }

class MouseDrag {
 public:
  MouseDrag() : dragged(false) {}

  bool dragged;
  // cv::Point is alias of cv::Point_<int>, has x, y coordinates
  cv::Point start, end;
};

// @param param: the optional parameter passed (from setMouseCallback()) to the
// @ callback function
void mouse_event_handler(int event, int x, int y, int flags, void *param) {
  // ! Use if (var == nullptr) rather than if (var)
  if (param == nullptr) return;
  MouseDrag *drag = (MouseDrag *)param;
  if (event == cv::EVENT_LBUTTONDOWN) {
    drag->dragged = true;
    drag->start = cv::Point(x, y);
    drag->end = cv::Point(0, 0);
  } else if (event == cv::EVENT_MOUSEMOVE) {
    if (drag->dragged) drag->end = cv::Point(x, y);
  } else if (event == cv::EVENT_MBUTTONUP) {
    if (drag->dragged) {
      drag->dragged = false;
      drag->end = cv::Point(x, y);
    }
  }
}

int main(int argc, char **argv) {
  // ! In C++11, constexpr cannot identify std::string, std::vector and others.
  // constexpr std::string input = "data/3dv_tutorial/daejeon_station.png";
  constexpr auto input = "data/3dv_tutorial/daejeon_station.png";
  // Given camera params: focal length, principal point (unit pixels)
  // , and camera height (unit meters)
  // * Use constexpr whenever possible
  constexpr double f = 810.5, cx = 480, cy = 270, L = 3.31;
  cv::Point3d cam_ori(deg2rad(-18.7), deg2rad(-8.2), deg2rad(2.0));
  // * Left inclusive while right exclusive
  cv::Range grid_x(-2, 3), grid_z(5, 35);

  // Load an image
  cv::Mat image = cv::imread(input);
  // Mat.empty() return true if the image(actually the Mat is an n-D array) has
  // no elements
  if (image.empty()) return -1;

  // Configure mouse callback
  MouseDrag drag;
  // cv::String is just alias of std::string
  std::string winname("Obejct Localization and Measurement");
  cv::namedWindow(winname);
  cv::setMouseCallback(winname, mouse_event_handler, &drag);

  // Draw grids on the ground
  cv::Matx33d K{f, 0, cx, 0, f, cy, 0, 0, 1};
  cv::Matx33d Rc = euler_angles_to_rotation_matrix(cam_ori);
  cv::Matx33d R = Rc.t();
  // ! Use Point3d to represent translation vector
  // ! Because some operators are not compatible between Point3d and Vec3d
  // ! e.g. plus +
  cv::Point3d tc{0, -L, 0};
  cv::Point3d t = -Rc.t() * tc;
  for (int z = grid_z.start; z <= grid_z.end; ++z) {
    // Perspective projection in regular coordinates
    // rather than homogeneous coordinates
    cv::Point3d p = K * (R * cv::Point3d(grid_x.start, 0, z) + t);
    cv::Point3d q = K * (R * cv::Point3d(grid_x.end, 0, z) + t);
    // image coordinates calculated from triangle similarity
    // * cv::Scalar is derived from cv::Vec, hence they can be converted back
    // * and forth
    cv::line(image, cv::Point2d(p.x / p.z, p.y / p.z),
             cv::Point2d(q.x / q.z, q.y / q.z), cv::Scalar(64, 128, 64));
  }
  for (int x = grid_x.start; x <= grid_x.end; x++) {
    cv::Point3d p = K * (R * cv::Point3d(x, 0, grid_z.start) + t);
    cv::Point3d q = K * (R * cv::Point3d(x, 0, grid_z.end) + t);
    cv::line(image, cv::Point2d(p.x / p.z, p.y / p.z),
             cv::Point2d(q.x / q.z, q.y / q.z), cv::Scalar(64, 128, 64));
  }

  while (true) {
    // * Copy constructor is way faster than clone and other copy methods
    // ! Copy constructor: No data is copied by these constructors. Instead, the
    // header pointing to m data or its sub-array is constructed and
    // associated with it. The reference counter, if any, is incremented. So,
    // when you modify the matrix formed using such a constructor, you also
    // modify the corresponding elements of m . If you want to have an
    // independent copy of the sub-array, use Mat::clone() .
    // cv::Mat image_copy(image);
    cv::Mat image_copy = image.clone();
    if (drag.end.x > 0 &&
        drag.end.y > 0) {  // This means the actual dragging operation happened.
      // Calculate object location and height
      // TODO In which frame?
      // ? * contact point in world frame
      cv::Point3d c =
          R.t() * cv::Point3d(drag.start.x - cx, drag.start.y - cy, f);
      // ? * head point in world frame
      cv::Point3d h = R.t() * cv::Point3d(drag.end.x - cx, drag.end.y - cy, f);
      double Z = c.z / c.y * L, X = c.x / c.y * L,
             H = (c.y / c.z - h.y / h.z) * Z;

      // Draw head / contact points and location / height
      cv::line(image_copy, drag.start, drag.end, cv::Scalar(0, 0, 255), 2);
      // * Negative thickness means filled circle
      // * Drawing start circle after end circle can show the start circle on
      // * top of the end circle on the picture, which in agree with the natural
      // * manner
      cv::circle(image_copy, drag.end, 4, cv::Scalar(255, 0, 0),
                 -1);  // red color
      cv::circle(image_copy, drag.start, 4, cv::Scalar(0, 255, 0),
                 -1);  // green
      cv::putText(image_copy, cv::format("X:%.2f, Z:%.2f, H:%.2f", X, Z, H),
                  drag.start + cv::Point(-20, 20), cv::FONT_HERSHEY_PLAIN, 1,
                  cv::Scalar(0, 255, 0));
    }

    // Show the result image
    cv::imshow(winname, image_copy);
    if (cv::waitKey(1) == 27) break;  // break if "ESC" is pressed
  }
  return 0;
}
