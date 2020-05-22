#include <algorithm>   // std::transform
#include <chrono>      // chrono::steady_clock etc.
#include <cmath>       // std::exp
#include <functional>  // std::plus
#include <iostream>    // std::cout, std::endl
#include <iterator>    // std::begin, std::end
#include <numeric>     // std::accumulate
#include <typeinfo>    // std::typeid, std::typeid.name
#include <utility>     // std::pair
#include <vector>      // std::vector

#include "Eigen/Dense"  // Eigen::Matrix3d, Eigen::Vector3d, Eigen::MatrixBase.ldlt(), Eigen::MatrixType.solve()
#include "opencv2/core.hpp"  // cv::RNG

using namespace std;

using Eigen::Matrix3d;
using Eigen::Vector3d;

using s_clock = chrono::steady_clock;
using m_secs = chrono::milliseconds;
using vd = vector<double>;
using dataset = vector<pair<double, double>>;

struct iter_result {
  // * Result of a single iteration of Gauss-Newton.
  double cost = 0;
  vd delta_beta;
};

class GaussNewton {
  // * Gauss-Newton method for params optimization.
 public:
  inline double func(vd &beta, const double x) {
    // * The Function whose params are to be estimated.
    const double y = exp(beta[0] * x * x + beta[1] * x + beta[2]);
    return y;
  }

  dataset generate_data(const int dataset_size, const double sigma) {
    // * Generate dataset according to the given params beta.
    // * The generated dataset was corruptd by gaussian noise.
    dataset data;
    vd beta{1., 2., 1.};  // beta: given params for generate data.
    cv::RNG rng;
    for (int i = 0; i < dataset_size; ++i) {
      double x = i / 100.;
      double y = func(beta, x) + rng.gaussian(sigma);
      data.emplace_back(x, y);
    }
    return data;
  }

  iter_result gauss_newton(const dataset &data, vd &beta, const double sigma) {
    // * A single iteration of Gauss-Newton.
    Matrix3d hessian = Matrix3d::Zero();
    Vector3d gradient = Vector3d::Zero();
    const double inv_sigma = 1. / sigma;
    double cost = 0;

    // TODO Use std::accumulate to refactor this loop.
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

    iter_result result;
    result.cost = cost;
    for (int i = 0; i < delta_beta.size(); ++i)
      result.delta_beta.push_back(delta_beta[i]);
    return result;
  }
};

int main(int argc, char *argv[]) {
  s_clock::time_point t1 = s_clock::now();

  // Gauss-Newton initialization
  GaussNewton gn;
  double sigma = 1.;
  int dataset_size = 100;
  auto data = gn.generate_data(dataset_size, sigma);
  int iterations = 20;
  double last_cost = 0.;

  // Estimands
  vd beta{2., -1., 5.};

  // Gauss-Newton starts
  for (int iter = 0; iter < iterations; ++iter) {
    auto result = gn.gauss_newton(data, beta, sigma);
    auto delta_beta = result.delta_beta;
    if (iter > 0 && result.cost >= last_cost) {
      cout << "Converged!" << endl;
      break;
    }
    std::transform(delta_beta.begin(), delta_beta.end(), beta.begin(),
                   beta.begin(), std::plus<double>());
    last_cost = result.cost;
    cout << "iter: " << iter + 1 << endl;
  }  // Gauss-Newton ends

  s_clock::time_point t2 = s_clock::now();
  m_secs time_span = chrono::duration_cast<m_secs>(t2 - t1);

  // Post-processing
  cout << "Time consumed: " << time_span.count() << " milliseconds" << endl;
  cout << "Estimated beta: ";
  for (auto param : beta) cout << param << " ";
  cout << "\nFinal cost: " << last_cost << endl;
  return 0;
}