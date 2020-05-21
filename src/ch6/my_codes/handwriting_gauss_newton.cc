#include <chrono>
#include <iostream>

#include "Eigen/Dense"
#include "opencv2/core.hpp"

// using namespace Eigen;
using namespace std;
using Eigen::MatrixXd;

using s_clock = chrono::steady_clock;
using secs = chrono::seconds;

int main(int argc, char *argv[]) {
  s_clock::time_point t1 = s_clock::now();
  MatrixXd m(2, 2);
  m(0, 0) = 1;
  m(0, 1) = 2;
  m(1, 0) = 3;
  m(1, 1) = 4;
  s_clock::time_point t2 = s_clock::now();
  secs time_span = chrono::duration_cast<secs>(t2 - t1);
  cout << m << endl;
  cout << time_span.count() << "seconds" << endl;
}