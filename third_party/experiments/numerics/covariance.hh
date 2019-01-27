#pragma once

#include "eigen.hh"

#include <vector>

// Compute the covariance of a list of vectors in a numerically stable manner
template <int DIM>
Eigen::Matrix<double, DIM, DIM> compute_covariance(const std::vector<Eigen::Matrix<double, DIM, 1>>& data) {
  using OutputMat = Eigen::Matrix<double, DIM, DIM>;
  using MeanVec   = Eigen::Matrix<double, DIM, 1>;

  const double num_pts_inv = 1.0 / static_cast<double>(data.size());

  MeanVec mean = MeanVec::Zero();
  for (const auto& pt : data) {
    mean += pt;
  }
  mean *= num_pts_inv;

  OutputMat cov;
  for (const auto& pt : data) {
    const MeanVec error = pt - mean;
    cov += error * error.transpose();
  }

  cov *= num_pts_inv;
  return cov;
}