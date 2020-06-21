#if !defined(KAHAN_SUM_UTILS_H_)
#define KAHAN_SUM_UTILS_H_

#include <iostream>
#include <numeric>  // std::accumulate
#include <vector>

template <typename T>
T kahan_sum(std::vector<T> input_array);

struct KahanAccumulator;  // Forward declaration
template <typename T>
KahanAccumulator kahan_sum_once(KahanAccumulator accumulated, T value);

#include "kahan_sum.hpp"

void kahan_sum_test() {
  std::vector<double> array{1, 0.01, 0.001, 0.0001, 0.000001, 0.00000000001};
  KahanAccumulator init;
  KahanAccumulator result =
      std::accumulate(array.begin(), array.end(), init, kahan_sum_once<double>);
  std::cout << "Normal sum: " << std::accumulate(array.begin(), array.end(), 0)
            << '\n';
  std::cout << "Kahan sum: " << kahan_sum<double>(array) << '\n';
  std::cout << "Kahan sum iter version: " << result.sum << '\n';
}

#endif  // KAHAN_SUM_UTILS_H_
