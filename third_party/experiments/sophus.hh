#pragma once

#include <sophus/se2.hpp>
#include <sophus/se3.hpp>
#include <sophus/sim3.hpp>

using SO3 = Sophus::SO3<double>;
using SE3 = Sophus::SE3<double>;
using Sim3 = Sophus::Sim3<double>;

using SO2 = Sophus::SO2<double>;
using SE2 = Sophus::SE2<double>;

namespace jcc {
inline SE3 exp_x(const double theta) {
  return SE3(SO3::exp(Eigen::Vector3d::UnitX() * theta), Eigen::Vector3d::Zero());
}
inline SE3 exp_y(const double theta) {
  return SE3(SO3::exp(Eigen::Vector3d::UnitY() * theta), Eigen::Vector3d::Zero());
}
inline SE3 exp_z(const double theta) {
  return SE3(SO3::exp(Eigen::Vector3d::UnitZ() * theta), Eigen::Vector3d::Zero());
}
}  // namespace jcc