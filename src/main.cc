
#include <iostream>

#include "Eigen/Dense"
#include "hello_slam.h"
#include "sophus/se3.hpp"

int main() {
    std::cout << "hello without libs" << std::endl;
    print_hello();
    return 0;
}