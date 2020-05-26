#ifndef SLAMBOOK_GENERATE_DATA_H_
#define SLAMBOOK_GENERATE_DATA_H_

#include <utility>  // std::pair
#include <vector>   // std::vector

class DataFunctor {
 public:
  DataFunctor(std::vector<double> beta);

  double operator()(const double x) const;

  std::vector<double> beta_;
};

using dataset = std::vector<std::pair<double, double>>;
dataset generate_data(DataFunctor* df, const int dataset_size,
                      const double sigma, const bool additive_noise);

#endif  // SLAMBOOK_GENERATE_DATA_H_