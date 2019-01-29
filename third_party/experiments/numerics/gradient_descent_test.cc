#include "numerics/gradient_descent.hh"

#include "numerics/numdiff.hh"

#include "testing/gtest.hh"

namespace numerics {

TEST(GradientDescent, does_it_work) {
  const Eigen::Matrix3d Q_u = Eigen::Matrix3d::Random();
  const Eigen::Matrix3d Q = (Q_u * Q_u.transpose());

  const auto cost_func = [Q](const Eigen::Vector3d &x) -> double {  //
    return x.transpose() * Q * x;
  };

  const Eigen::Vector3d offset = Eigen::Vector3d::Ones();
  const auto gd_cost_func = [cost_func, offset](const Eigen::VectorXd &x, Eigen::VectorXd *grad) {  //
    const double cost = cost_func(x - offset);
    if (grad) {
      *grad = numerical_gradient(Eigen::Vector3d(x - offset), cost_func);
    }
    return cost;
  };

  const Eigen::VectorXd result = minimize_gradient_descent(Eigen::Vector3d::Random() * 500, gd_cost_func);

  std::cout << result.transpose() << std::endl;
  std::cout << offset.transpose() << std::endl;

  constexpr double EPS = 1e-5;
  EXPECT_LT((result - offset).squaredNorm(), EPS);
}
}  // namespace numerics
