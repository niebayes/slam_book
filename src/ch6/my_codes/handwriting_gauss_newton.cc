#include <algorithm>  // std::transform
#include <chrono>     // chrono::steady_clock etc.
#include <cmath>      // std::exp
#include <iostream>   // std::cout, std::endl
#include <iterator>   // std::begin, std::end
#include <numeric>    // std::accumulate
#include <typeinfo>   // std::typeid, std::typeid.name
#include <utility>    // std::pair
#include <vector>     // std::vector

#include "Eigen/Dense"
#include "opencv2/core.hpp"

using namespace std;
using Eigen::Matrix3d;
using Eigen::Vector3d;

using s_clock = chrono::steady_clock;
using m_secs = chrono::milliseconds;
using vd = vector<double>;

class GaussNewton {
 private:
  static double sigma = 1.;

 public:
  inline double func(const vd &beta, double x) {
    if (beta.size() != 3) return -1.;
    double y = exp(beta[0] * x * x + beta[1] * x + beta[2]);
    return y;
  }

  vector<pair<double, double>> generate_data(const int n,
                                             const double sigma = sigma) {
    vector<pair<double, double>> data;
    const vd beta{1., 2., 1.};
    cv::RNG rng;
    for (int i = 0; i < n; ++i) {
      double x = i / 100.;
      double y = func(beta, x) + rng.gaussian(sigma);
      data.emplace_back(x, y);
    }
    return data;
  }

  Vector3d gauss_newton(double &cost, double &last_cost,
                        const vector<pair<double, double>> &data,
                        const double sigma, vd &beta) {
    Matrix3d hessian = Matrix3d::Zero();
    Vector3d gradient = Vector3d::Zero();
    // vd beta{2., -1., 5.};
    const double inv_sigma = 1. / sigma;

    cost = 0.;
    for (auto it = data.begin(); it < data.end(); ++it) {
      auto x = it->first, y = it->second;
      double residual = y - func(beta, x);

      Vector3d jacobian{-x * x * func(beta, x), -x * func(beta, x),
                        -func(beta, x)};

      hessian += inv_sigma * inv_sigma * jacobian * jacobian.transpose();
      gradient += -inv_sigma * inv_sigma * residual * jacobian;

      cost += residual * residual;
    }
    auto delta_beta = hessian.ldlt().solve(gradient);
    return delta_beta;
  }
};

int main(int argc, char *argv[]) {
  s_clock::time_point t1 = s_clock::now();
  GaussNewton gn;
  auto data = gn.generate_data(100, 1.);
  int iterations = 20;
  double cost = 0., last_cost = 0.;
  double sigma = 1.;
  vd beta{2., -1., 5.};

  for (int iter = 0; iter < iterations; ++iter) {
    auto delta_beta = gn.gauss_newton(cost, last_cost, data, sigma, beta);
    beta[0] += delta_beta[0];
    beta[1] += delta_beta[1];
    beta[2] += delta_beta[2];
    last_cost = cost;
  }

  s_clock::time_point t2 = s_clock::now();
  m_secs time_span = chrono::duration_cast<m_secs>(t2 - t1);
  cout << time_span.count() << "milliseconds" << endl;
  cout << "Estimated beta: ";
  for (auto param : beta) cout << param << " ";
  return 0;
}