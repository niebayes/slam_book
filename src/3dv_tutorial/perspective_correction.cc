#include <initializer_list>
#include <ostream>
#include <vector>

#include "opencv2/opencv.hpp"

void mouse_event_handler(int event, int x, int y, int flags, void *param) {
  if (event == cv::EVENT_LBUTTONDOWN) {
    // Add the point to the given vector
    std::vector<cv::Point> *points_src = (std::vector<cv::Point> *)param;
    points_src->push_back(cv::Point(x, y));
    std::cout << cv::format("A point (index: %zd) is selected at (%i, %i).\n",
                            points_src->size() - 1, x, y);
  }
}

int main() {
  constexpr auto input = "data/3dv_tutorial/images/sunglok_desk.jpg";
  cv::Size card_size(450, 250);

  // Prepare the rectified points
  std::vector<cv::Point> points_dst{{0, 0},
                                    {card_size.width, 0},
                                    {0, card_size.height},
                                    {card_size.width, card_size.height}};

  // Load an image
  cv::Mat original = cv::imread(input);
  if (original.empty()) return -1;

  // Get the matched points from the user's mouse
  std::vector<cv::Point> points_src;
  cv::String winname("Perspective Correction");
  cv::setMouseCallback(winname, mouse_event_handler, &points_src);
  while (points_src.size() < 4) {
    cv::Mat display = original.clone();
  }
  return 0;
}