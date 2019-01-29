#pragma once

#include "out.hh"
#include "types.hh"

#include "eigen.hh"
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <sophus/se2.hpp>
#include <sophus/so2.hpp>

namespace raytrace {
using Vec1 = Vec<1>;
using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using Mat2 = Eigen::Matrix2d;

double cost(const Vec3 &eta, const std::vector<Vec2> &x, const std::vector<Vec2> &z) {
  const se2 T = se2::exp(eta);

  double error_sq = 0.0;
  for (std::size_t k = 0; k < x.size(); ++k) {
    const Vec2 error = (T * x[k]) - z[k];
    error_sq += error.squaredNorm();
  }

  return error_sq;
}

Vec2 error(const Vec3 &eta, const std::vector<Vec2> &x, const std::vector<Vec2> &z) {
  const se2 T = se2::exp(eta);

  // double error_sq = 0.0;
  Vec2 error_sq = Vec2::Zero();
  for (std::size_t k = 0; k < x.size(); ++k) {
    const Vec2 error = (T * x[k]) - z[k];
    error_sq += error.array().square().matrix();
  }

  return error_sq;
}

struct dExpSe2 {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix2d Rprime;
  Eigen::Matrix2d V;
  Eigen::Matrix2d Vprime;
  Vec3            eta;

  Mat<2, 3> jacobian_from_y(const Vec2 &y) const {
    Mat<2, 3> m;
    m.setZero();
    m.block<2, 2>(0, 0) = V;
    m.block<2, 1>(0, 2) = (Rprime * y) + (Vprime * eta.head<2>());
    return m;
  }
};

void dexp_se2(const Vec3 &eta, Out<dExpSe2> jac) {
  // todo: Use taylor exp instead of explicit sincos for stability near zero
  const double c          = std::cos(eta(2));
  const double s          = std::sin(eta(2));
  const double inv_theta  = 1 / eta(2);
  const double inv_theta2 = inv_theta * inv_theta;

  const double cos_inv_theta  = c * inv_theta;
  const double sin_inv_theta  = s * inv_theta;
  const double sin_inv_theta2 = sin_inv_theta * inv_theta;

  const double cos_m1_invtheta2 = (c - 1) * inv_theta2;

  Eigen::Matrix2d v;
  v.row(0) << s, (c - 1);
  v.row(1) << 1 - c, s;
  v *= inv_theta;

  Eigen::Matrix2d dv_dtheta;
  dv_dtheta.row(0) << cos_inv_theta - (sin_inv_theta2), -sin_inv_theta - (cos_m1_invtheta2);
  dv_dtheta.row(1) << sin_inv_theta + cos_m1_invtheta2, cos_inv_theta - sin_inv_theta2;

  Eigen::Matrix2d dR_dtheta;
  dR_dtheta.row(0) << -s, -c;
  dR_dtheta.row(1) << c, -s;

  jac->Rprime = dR_dtheta;
  jac->V      = v;
  jac->Vprime = dv_dtheta;
  jac->eta    = eta;
}

Vec3 dcost(const Vec3 &eta, const std::vector<Vec2> &x, const std::vector<Vec2> &z) {
  const se2 T = se2::exp(eta);
  dExpSe2   preJ;
  dexp_se2(eta, out(preJ));

  Vec3 derror_sq_total = Vec3::Zero();
  for (std::size_t k = 0; k < x.size(); ++k) {
    const Vec2 error = (T * x[k]) - z[k];

    const Mat<2, 3> derror_deta = preJ.jacobian_from_y(x[k]);
    const Vec3 derror_sq = (2 * error.transpose()) * derror_deta;
    derror_sq_total += derror_sq;
  }

  return derror_sq_total;
}

template <int cols, typename Callable1, typename Callable2>
Eigen::Matrix<double, cols, 1> gauss_newton(const Eigen::Matrix<double, cols, 1> &x0,
                                            const Callable1 &fcn,
                                            const Callable2 &jac,
                                            const double     feps      = 1e-6,
                                            const int        max_iters = 15) {
  using VecX     = Eigen::Matrix<double, 1, cols>;
  using Gradient = Eigen::Matrix<double, cols, 1>;
  VecX x         = x0;

  double last_cost = fcn(x);
  for (int k = 0; k < max_iters; ++k) {
    // todo: llt
    const Gradient J = jac(x);
    const VecX     c = (J.transpose() * J).inverse() * J.transpose() * fcn(x);
    x                = x - c;
    double new_cost  = fcn(x);

    if (std::abs(new_cost - last_cost) < feps) {
      break;
    }

    last_cost = new_cost;
  }
  return x;
}
}