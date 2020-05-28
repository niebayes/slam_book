#include "generate_data.h"

#include <cmath>       // std::exp
#include <functional>  // std::function
#include <utility>     // std::pair
#include <vector>      // std::vector

#include "opencv2/core/core.hpp"  // cv::RNG

DataFunctor::DataFunctor(std::vector<double> beta) : beta_(beta) {}

double DataFunctor::operator()(const double x) const {
  double y = std::exp(beta_[0] * x * x + beta_[1] * x + beta_[2]);
  return y;
}

using dataset = std::vector<std::pair<double, double>>;
dataset generate_data(const std::vector<double>& beta, const int dataset_size,
                      const double sigma, const bool additive_noise) {
  // * Generate dataset according to the given params beta.
  // * The generated dataset was corruptd by gaussian noise.
  dataset data;
  DataFunctor func(beta);
  cv::RNG rng;
  for (int i = 0; i < dataset_size; ++i) {
    double x = i / 100.;
    double y = additive_noise ? func(x) + rng.gaussian(sigma) : func(x);
    data.emplace_back(x, y);
  }
  return data;
}