#include "numerics/optimize.hh"

#include "testing/gtest.hh"

#include "numerics/numdiff.hh"

namespace numerics {

TEST(OptimizeTest, minimizes_gradient_descent) {
  const Eigen::Matrix3d Q_u = Eigen::Matrix3d::Random();
  const Eigen::Matrix3d Q = (Q_u * Q_u.transpose());

  const auto cost_func = [Q](const Eigen::Vector3d &x) -> double {  //
    return x.transpose() * Q * x;
  };

  const Eigen::Vector3d offset = Eigen::Vector3d::Ones();
  const auto gd_cost_func = [cost_func, offset](
                                const Eigen::VectorXd &x,
                                Eigen::VectorXd *const grad,
                                Eigen::MatrixXd *const hessian_unused) {  //
    const double cost = cost_func(x - offset);
    if (grad) {
      *grad = numerical_gradient(Eigen::Vector3d(x - offset), cost_func);
    }
    return cost;
  };

  OptimizationProblem problem;
  problem.objective = gd_cost_func;
  OptimizationState initialization;
  initialization.x = Eigen::Vector3d::Random();
  initialization.lambda = Eigen::Vector3d::Zero();

  std::cout << offset.transpose() << std::endl;

  const auto result =
      optimize<ObjectiveMethod::kGradientDescent, ConstraintMethod::kAugLag>(
          initialization, problem);
}
}  // namespace numerics
