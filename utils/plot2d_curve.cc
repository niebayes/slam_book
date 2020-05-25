#include "plot2d_curve.h"

#include <cmath>    // std::exp
#include <utility>  // std::pair

#include "opencv2/highgui.hpp"
#include "opencv2/plot.hpp"

using dataset = std::vector<std::pair<double, double>>;
void plot2d_curve(const dataset& ds) {
  std::vector<double> data_x, data_y;
  for (auto& p : ds) {
    data_x.push_back(p.first);
    data_y.push_back(p.second);
  }
  auto curve = cv::plot::Plot2d::create(data_x, data_y);
  cv::Mat image;
  curve->render(image);
  cv::imshow("Function curve", image);
  cv::waitKey(0);
}