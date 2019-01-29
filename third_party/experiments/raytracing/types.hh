#pragma once

#include <Eigen/StdVector>
#include <sophus/se2.hpp>
#include <sophus/so2.hpp>

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector2d)

using Scalar = double;
using so2    = Sophus::SO2<Scalar>;
using se2    = Sophus::SE2<Scalar>;

template <int rows, int cols>
using Mat = Eigen::Matrix<double, rows, cols>;

template <int cols>
using Vec = Eigen::Matrix<double, 1, cols>;