#include "raytracing/optimization.hh"
#include "numerics/numdiff.hh"

#include "testing/gtest.hh"

using Vec1   = Eigen::Matrix<double, 1, 1>;
using Vec2   = Eigen::Vector2d;
using Vec3   = Eigen::Vector3d;
namespace rt = raytrace;
namespace nm = numerics;

// Numerical gradient of cost
//
TEST(CostGradient, numerical) {
  constexpr double EPS = 1e-3;

  //
  // Setup
  //

  const double                    theta = 0.5;
  const Eigen::Rotation2D<double> R(theta);

  std::vector<Vec2> x;
  std::vector<Vec2> z;

  for (int k = 0; k < 1; ++k) {
    const Vec2 v(k * 0.5, -0.5 * k);
    x.emplace_back(v);
    z.emplace_back(R * v);
  }

  const auto fcn = std::bind(rt::cost, std::placeholders::_1, x, z);

  //
  // Action
  //

  const Vec3 eta(0.1, 0.1, theta);
  const Vec3 analytical_dcost = rt::dcost(eta, x, z);

  //
  // Verification
  //

  const Vec3 numerical_dcost = nm::numerical_gradient(eta, fcn);
  EXPECT_LT((numerical_dcost - analytical_dcost).norm(), EPS);
}

// Align points via GN
// (In principle: The "align" step of icp)
//
TEST(Optimization, GaussNewton) {
  constexpr double EPS = 1e-3;

  //
  // Setup
  //

  const double                    theta(0.5);
  const Vec2                      trans(0.3, -0.4);
  const Eigen::Rotation2D<double> R(theta);

  std::vector<Vec2> x;
  std::vector<Vec2> z;

  for (int k = -1; k < 3; ++k) {
    const Vec2 v(k * 0.5, -0.5 * k);
    x.emplace_back(v);
    z.emplace_back((R * v) + trans);
  }

  const auto fcn = std::bind(rt::cost, std::placeholders::_1, x, z);
  const auto jac = std::bind(rt::dcost, std::placeholders::_1, x, z);

  //
  // Action
  //

  const Vec3 eta(0.0, 0.0, 0.1);
  const Vec3 result = rt::gauss_newton<3>(eta, fcn, jac);

  //
  // Verification
  //

  const Vec2   r_trans = se2::exp(result).translation();
  const double r_theta = se2::exp(result).so2().log();
  EXPECT_NEAR(r_theta, theta, EPS);
  EXPECT_LT((r_trans - trans).norm(), EPS);
}
