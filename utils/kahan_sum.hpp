// ! There's no need to compile header files, .h or .hpp

template <typename T>
T kahan_sum(std::vector<T> input_array) {
  T sum = 0, compensation = 0;
  for (auto i = std::begin(input_array); i < std::end(input_array); ++i) {
    T higher_orders = *i - compensation;
    T t = sum + higher_orders;
    compensation = (t - sum) - higher_orders;
    sum = t;
  }
  return sum;
}

struct KahanAccumulator {
  double sum = 0;
  double compensation = 0;
};

template <typename T>
KahanAccumulator kahan_sum_once(KahanAccumulator accumulated, T value) {
  KahanAccumulator result;
  T y = value - accumulated.compensation;
  T t = accumulated.sum + y;
  result.compensation = (t - accumulated.sum) - y;
  result.sum = t;
  return result;
}

