#include <chrono>
#include <cmath>
#include <iostream>
#include <vector>

#include "Eigen/Core"
#include "g2o/core/base_unary_edge.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/g2o_core_api.h"
#include "g2o/core/optimization_algorithm_dogleg.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "generate_data.h"
#include "opencv2/core/core.hpp"
#include "plot2d_curve.h"

using namespace std;

class CurveFittingVertex : public g2o::BaseVertex<3, Eigen::Vector3d> {
 public:
  // ! cf.
  // https://eigen.tuxfamily.org/dox-devel/group__TopicStructHavingEigenMembers.html
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  virtual void setToOriginImpl() override { _estimate << 0, 0, 0; }

  // ! In order to override, the two sets of signature of the functions should
  // ! be matched perfectly.
  virtual void oplusImpl(const double* update) override {
    _estimate += Eigen::Vector3d(update);
  }

  virtual bool read(std::istream& in) {}

  virtual bool wirte(std::ostream& out) const {}
};

class CurveFittingEdge
    : public g2o::BaseUnaryEdge<1, double, CurveFittingVertex> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CurveFittingEdge(double x) : BaseUnaryEdge(), x_(x) {}

  virtual void computeError() override {
    const CurveFittingVertex* v =
        static_cast<const CurveFittingVertex*>(_vertices[0]);
    const Eigen::Vector3d beta = v->estimate();
    _error(0, 0) =
        _measurement - std::exp(beta[0] * x_ * x_ + beta[1] * x_ + beta[2]);
  }

  virtual void linearizeOplus() override {
    const CurveFittingVertex* v =
        static_cast<const CurveFittingVertex*>(_vertices[0]);
    const Eigen::Vector3d beta = v->estimate();
    double y = exp(beta[0] * x_ * x_ + beta[1] * x_ + beta[2]);
    _jacobianOplusXi[0] = -x_ * x_ * y;
    _jacobianOplusXi[1] = -x_ * y;
    _jacobianOplusXi[2] = -y;
  }

  virtual bool read(std::istream& in) {}
  virtual bool write(std::ostream& out) const {}

 public:
  double x_;
};

int main(int argc, char** argv) {
  // * Generate dataset
  vector<double> real_beta{1., 2.,
                           1.};  // beta: given params for generate data.
  int dataset_size = 100;
  double sigma = 1.;
  bool additive_noise = true;
  DataFunctor func(real_beta);
  auto dataset = generate_data(&func, dataset_size, sigma, additive_noise);
  plot2d_curve(dataset);

  // * Estimands
  vector<double> beta{2., -1., 5.};

  // * Set up g2o:
  // * 设置优化对象的维度
  // * 使用线性求解，并结社线性求解器类型
  using BlockSolverType = g2o::BlockSolver<g2o::BlockSolverTraits<3, 1>>;
  using LinearSolverType =
      g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>;

  // * Set up optimization method:
  // * Gradient method: GN, LM, DogLeg
  auto solver = new g2o::OptimizationAlgorithmGaussNewton(
      g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(true);

  // * 往g2o的graph中添加节点
  CurveFittingVertex* v = new CurveFittingVertex();
  v->setEstimate(Eigen::Vector3d(beta[0], beta[1], beta[2]));
  v->setId(0);
  optimizer.addVertex(v);

  // * 往g2o的graph中添加边
  for (int i = 0; i < dataset_size; ++i) {
    CurveFittingEdge* edge = new CurveFittingEdge(dataset[i].first);
    edge->setId(i);
    edge->setVertex(0, v);
    edge->setMeasurement(dataset[i].second);
    edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * 1. / sigma);
    optimizer.addEdge(edge);
  }

  // * Start optimization
  auto t1 = chrono::steady_clock::now();
  optimizer.initializeOptimization();
  optimizer.optimize(10);
  auto t2 = chrono::steady_clock::now();
  auto time_span = chrono::duration_cast<chrono::milliseconds>(t2 - t1);
  cout << "Time comsumed: " << time_span.count() << " milliseconds" << endl;

  // * 输出优化结果
  Eigen::Vector3d optimized_beta = v->estimate();
  cout << "Optimized params are: " << optimized_beta.transpose() << endl;

  return 0;
}