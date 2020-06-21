#ifndef CONST_ITER_UTILS_H_
#define CONST_ITER_UTILS_H_

#include <iostream>
#include <iterator>
#include <vector>

namespace std {

template <typename C>
constexpr auto cbegin(const C &container) -> decltype(std::begin(container));

template <typename C>
constexpr auto cend(const C &container) -> decltype(std::end(container));

#include "const_iter.hpp"

}  // namespace std

void const_iter_test() {
  std::vector<int> ints{1, 2, 3};
  for (auto i = std::begin(ints); i < std::end(ints); ++i) {
    try {
      *i += 1;
      std::cout << "Non-const version test passed!\n";
    } catch (const std::exception &e) {
      std::cout << "Non-const version test not passed!\n";
    }
  }
  for (auto i = std::cbegin(ints); i < std::cend(ints); ++i) {
    try {
      // TODO why can't i catch this error
      *i += 1;
      std::cout << "Const version test not passed!\n";
    } catch (const std::exception &e) {
      std::cout << "Const version test passed!\n";
    }
  }
}

#endif  // CONST_ITER_UTILS_H_
