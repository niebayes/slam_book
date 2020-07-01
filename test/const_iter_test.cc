#include "const_iter.h"

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

int main(int argc, char **argv) {
  const_iter_test();
  return 0;
}