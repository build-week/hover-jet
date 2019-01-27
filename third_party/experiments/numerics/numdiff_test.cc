#include "numerics/numdiff.hh"
#include "numerics/num_hessian.hh"

#include "eigen.hh"

#include "testing/gtest.hh"

using Vec1 = Eigen::Matrix<double, 1, 1>;
using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
namespace numerics {
namespace {
double gfunc(const Vec2& z) {
  return z(0) * z(0) + 5 * z(1);
}

Vec2 zfunc(const Vec3& a) {
  return Vec2(a(0) * a(1), 4.0 * a(2));
}

MatNd<2, 3> dzfunc_da(const Vec3& a) {
  MatNd<2, 3> J;

  J.row(0) << a(1), a(0), 0.0;
  J.row(1) << 0.0, 0.0, 4.0;
  return J;
}
}  // namespace

TEST(NumericalGradient, gfunc) {
  constexpr double EPS = 1e-6;

  //
  // Setup
  //

  const Vec2 x(1.0, 0.0);
  const Vec2 expected(2.0, 5.0);

  //
  // Action
  //

  const Vec2 J = numerical_gradient(x, gfunc);

  //
  // Verification
  //

  // max error
  EXPECT_LT((J - expected).lpNorm<Eigen::Infinity>(), EPS);
}

TEST(NumericalJacobian, zfunc) {
  const std::vector<Vec3> test_pts = {Vec3(0.0, 0.0, 0.0), Vec3(1.0, -2.0, 7.0),
                                      Vec3(9.0, -12.0, 0.0), Vec3(1.0, 2.0, 1.0)};

  for (const auto x : test_pts) {
    const MatNd<2, 3> J = numerical_jacobian<2>(x, zfunc);
    const MatNd<2, 3> expected_J = dzfunc_da(x);

    constexpr double EPS = 1e-3;
    EXPECT_LT((J - expected_J).norm(), EPS);
  }
}
TEST(NumericalHessian, ufunc) {
  const std::vector<Vec3> test_pts = {Vec3(0.0, 0.0, 0.0), Vec3(1.0, -2.0, 7.0),
                                      Vec3(9.0, -12.0, 0.0), Vec3(1.0, 2.0, 1.0)};

  using TMat = Eigen::Matrix<double, 3, 3>;
  const TMat A_u = TMat::Random();
  const TMat hessian = A_u + A_u.transpose();

  const auto test_func = [hessian](const Vec3& x) -> double {  //
    return (0.5 * x.transpose() * hessian * x);
  };
  for (const auto x : test_pts) {
    const TMat numerical_hess = numerical_hessian(x, test_func);
    constexpr double EPS = 0.05;
    EXPECT_LT((numerical_hess - hessian).norm(), EPS);
  }
}
}  // namespace numerics
