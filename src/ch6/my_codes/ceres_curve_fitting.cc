#include <iostream>
#include <vector>

#include "ceres/ceres.h"
#include "opencv2/core.hpp"
// #include "glog/logging.h"  // Google Logging Library

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

using cv::RNG;

class CostFunctor {
 public:
  CostFunctor(double x, double y) : x_(x), y_(y) {}

  template <typename T>
  bool operator()(const T* const a, const T* const b, const T* const c,
                  T* residual) const {
    residual[0] =
        T(y_) - ceres::exp(a[0] * T(x_) * T(x_) + b[0] * T(x_) + c[0]);
    return true;
  }

 private:
  double x_, y_;
};

int main(int argc, char** argv) {
  double real_a = 1., real_b = 2., real_c = 1.;
  double initial_a = 2., initial_b = -1., initial_c = 5.;
  int kN = 100;
  double sigma = 1.;
  double inv_sigma = 1. / sigma;
  RNG rng;

  std::vector<double> data_x, data_y;
  for (int i = 0; i < kN; ++i) {
    double x = i / 100.;
    data_x.push_back(x);
    data_y.push_back(exp(real_a * x * x + real_b * x + real_c) +
                     rng.gaussian(sigma));
  }

  Problem problem;
  double a = initial_a, b = initial_b, c = initial_c;
  for (int i = 0; i < kN; ++i) {
    CostFunction* cost_function =
        new AutoDiffCostFunction<CostFunctor, 1, 1, 1, 1>(
            new CostFunctor(data_x[i], data_y[i]));
    problem.AddResidualBlock(cost_function, nullptr, &a, &b, &c);
  }

  Solver::Options options;
  options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
  options.minimizer_progress_to_stdout = true;
  Solver::Summary summary;
  Solve(options, &problem, &summary);

  std::cout << summary.BriefReport() << "\n";
  std::cout << "Estimated params are: \n";
  std::cout << a << " " << b << " " << c;
  return 0;
}