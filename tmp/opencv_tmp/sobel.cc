//@cf. Simple but inituitive Sobel implementation
// https://www.programming-techniques.com/2013/03/sobel-and-prewitt-edge-detector-in-c-image-processing.html

//@cf. The most helpful Sobel material found thus far
// http://homepages.inf.ed.ac.uk/rbf/HIPR2/sobel.htm
#include <cmath>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

// Computes the x component of the gradient vector
// at a given point in a image.
// returns gradient in the x direction
int xGradient(Mat image, int x, int y) {
  return image.at<uchar>(y - 1, x - 1) + 2 * image.at<uchar>(y, x - 1) +
         image.at<uchar>(y + 1, x - 1) - image.at<uchar>(y - 1, x + 1) -
         2 * image.at<uchar>(y, x + 1) - image.at<uchar>(y + 1, x + 1);
}

// Computes the y component of the gradient vector
// at a given point in a image
// returns gradient in the y direction

int yGradient(Mat image, int x, int y) {
  return image.at<uchar>(y - 1, x - 1) + 2 * image.at<uchar>(y - 1, x) +
         image.at<uchar>(y - 1, x + 1) - image.at<uchar>(y + 1, x - 1) -
         2 * image.at<uchar>(y + 1, x) - image.at<uchar>(y + 1, x + 1);
}

int main() {
  Mat src, dst;
  int gx, gy, sum;

  // Load an image
  src = imread("data/3dv_book/images/bikesgray.jpg", IMREAD_GRAYSCALE);
  dst = src.clone();
  if (!src.data) {
    return -1;
  }

  for (int y = 0; y < src.rows; y++)
    for (int x = 0; x < src.cols; x++) dst.at<uchar>(y, x) = 0.0;

  for (int y = 1; y < src.rows - 1; y++) {
    for (int x = 1; x < src.cols - 1; x++) {
      gx = xGradient(src, x, y);
      gy = yGradient(src, x, y);
      sum = abs(gx) + abs(gy);
      sum = sum > 255 ? 255 : sum;
      sum = sum < 0 ? 0 : sum;
      dst.at<uchar>(y, x) = sum;
    }
  }

  namedWindow("final");
  imshow("final", dst);

  namedWindow("initial");
  imshow("initial", src);

  waitKey();

  return 0;
}