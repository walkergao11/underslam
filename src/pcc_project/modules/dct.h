#include <stdio.h>
#include <fftw3.h>
#include <time.h>
#include <fftw3.h>
#include <vector>
#include <utility>
#include <iostream>
#include <algorithm>
#include "config.h"
// include opencv
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
//using namespace std;
// Assuming Matrix is defined as follows
namespace pcc {

    // 将 Matrix 的定义移入命名空间
    template <typename T>
    using Matrix = std::vector<std::vector<T>>; // 推荐使用 std::vector

    // 将所有函数声明也移入命名空间
    std::pair<Matrix<double>, Matrix<bool>> sadct(const Matrix<double> &data, const Matrix<bool> &mask);
    Matrix<double> saidct(const Matrix<double> &data, const Matrix<bool> &mask);
    std::pair<Matrix<double>, Matrix<bool>> delta_sadct(const Matrix<double> &data, const Matrix<bool> &mask);
    Matrix<double> delta_saidct(const Matrix<double> &data, const Matrix<bool> &mask);

}