#include "fluids/navier_stokes_2d.hh"

#include "fluids/fields_2d.hh"

#include "testing/gtest.hh"

namespace jcc {
namespace fluids {
namespace plane {

TEST(JacobiTest, actually_converges) {
  // The natural numbers, idealized as "the natches" can be freely chosen for this parameter
  constexpr int DIMS = 15;

  const Eigen::MatrixXd x0 = Eigen::MatrixXd::Ones(DIMS, DIMS);

  //////////////////////////////////////////////////////////////////
  Eigen::MatrixXd b = Eigen::MatrixXd::Ones(DIMS, DIMS) * 0.5;
  b.rightCols(2) += Eigen::MatrixXd::Ones(DIMS, 2) * 2.0;
  b(3, 3)                           = 9.0;
  b.bottomRightCorner(5, 5).array() = -3.0;
  //////////////////////////////////////////////////////////////////

  // Find an 'x' such that laplace(x) = b
  const auto cfg = JacobiConfig({
      .alpha         = -1e-1,  //
      .beta          = 4.0,    //
      .max_iters     = -1,     //
      .min_rel_delta = 1e-6    //
  });
  const auto x   = solve_poisson_jacobi_image(x0, b, cfg);

  constexpr double      SQRT_01             = std::sqrt(0.1);
  const Eigen::MatrixXd numerical_laplacian = plane::compute_laplacian(x, SQRT_01);

  constexpr double EPS = 1e-5;

  std::cout << "Solution: " << std::endl;
  std::cout << x << std::endl;

  std::cout << "b: " << std::endl;
  std::cout << b << std::endl;

  std::cout << "Numerical: " << std::endl;
  std::cout << numerical_laplacian << std::endl;

  EXPECT_LT((numerical_laplacian - b).norm() / (DIMS * DIMS), EPS);
}

}  // namespace plane
}  // namespace fluids
}  // namespace jcc