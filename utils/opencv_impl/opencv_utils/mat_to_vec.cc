
//@cf.
// https://answers.opencv.org/question/145214/convert-cvmat-to-stdvector-without-copying/
cv::Mat res(m, n, CV_32FC1);
std::vector<float> vec(res.begin<float>(), res.end<float>());

std::vector<float> vec;
cv::Mat res(m, n, CV_32FC1);
vec.assign(res.begin<float>(), res.end<float>());

//@cf.
// https://stackoverflow.com/a/26685567/11240780
std::vector<float> array;
if (mat.isContinuous()) {
  // array.assign((float*)mat.datastart, (float*)mat.dataend); // <- has
  // problems for sub-matrix like mat = big_mat.row(i)
  array.assign((float*)mat.data,
               (float*)mat.data + mat.total() * mat.channels());
} else {
  for (int i = 0; i < mat.rows; ++i) {
    array.insert(array.end(), mat.ptr<float>(i),
                 mat.ptr<float>(i) + mat.cols * mat.channels());
  }
}