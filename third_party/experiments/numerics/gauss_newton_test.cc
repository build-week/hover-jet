#include "numerics/gauss_newton.hh"

#include "eigen.hh"
#include "numerics/numdiff.hh"
#include "out.hh"

#include "testing/gtest.hh"

#include <iomanip>

namespace numerics {
constexpr int IN_DIM = 3;
constexpr int OUT_DIM = 3;

using InVec = VecNd<IN_DIM>;
using OutVec = VecNd<OUT_DIM>;
using ErrorJacobian = MatNd<OUT_DIM, IN_DIM>;

OutVec pure_error_fcn(const InVec& x) {
  using UJac = MatNd<2, IN_DIM>;
  const static UJac H = UJac::Random();
  OutVec v = OutVec::Zero();
  v.head<2>() = -H * x;
  return v;
}

OutVec error_fcn(const InVec& x, ErrorJacobian* jac) {
  *jac += numerical_jacobian<OUT_DIM>(x, pure_error_fcn);
  return pure_error_fcn(x);
}

TEST(GaussNewton, error_printouts) {
  const VecNd<IN_DIM> init = VecNd<IN_DIM>::Random();

  const OptimizationConfiguration config;

  using ThisError = ErrorFunction<IN_DIM, OUT_DIM>;
  const auto erf_casted = static_cast<ThisError>(error_fcn);
  const std::vector<ThisError> errors{erf_casted};
  const auto result = gauss_newton_minimize(errors, init, config);
  std::cout << std::boolalpha << result.success << std::endl;
  std::cout << "soln : " << result.solution.transpose() << std::endl;
  std::cout << "res  : " << result.terminal_error.transpose() << std::endl;
}

using ThisError = ErrorFunction<IN_DIM, OUT_DIM>;
void add_constraint(const double weight, const Eigen::Vector3d constraint_normal, Out<std::vector<ThisError>> errors) {
  const auto constraint = [constraint_normal](const InVec& x) {
    OutVec v = OutVec::Zero();
    v.tail<1>() = VecNd<1>(5.0 - constraint_normal.dot(x));
    return v;
  };

  const ThisError constraint_with_jacobian = [weight, constraint](const InVec& x, ErrorJacobian* jac) {
    if (jac) {
      const auto J = numerical_jacobian<OUT_DIM>(x, constraint);
      *jac += (weight * J);
    }
    return weight * constraint(x);
  };
  errors->push_back(constraint_with_jacobian);
}

TEST(PenaltyMethodGaussNewton, constraints) {
  VecNd<IN_DIM> init = VecNd<IN_DIM>::Random();
  const OptimizationConfiguration config;

  for (int k = 0; k < 2; ++k) {
    const auto erf_casted = static_cast<ThisError>(error_fcn);

    std::vector<ThisError> errors{erf_casted};
    const double c_weight = k * 10.0;
    const Eigen::Vector3d constraint_normal1 = Eigen::Vector3d(1.0, 0.0, 0.0);
    add_constraint(c_weight, constraint_normal1, out(errors));
    // const Eigen::Vector3d constraint_normal2 = Eigen::Vector3d(0.0, 1.0, 0.0);
    // add_constraint(c_weight, constraint_normal2, out(errors));

    const auto result = gauss_newton_minimize(errors, init, config);
    init = result.solution;

    std::cout << "\n--" << std::endl;
    std::cout << "done : " << std::boolalpha << result.success << std::endl;
    std::cout << "soln : " << result.solution.transpose() << std::endl;
    std::cout << "viol1 : " << errors.at(1)(result.solution, nullptr).transpose() << std::endl;
    // std::cout << "viol2 : " << constraint_normal2.transpose() * result.solution << std::endl;
    std::cout << "res  : " << result.terminal_error.transpose() << std::endl;
  }
}
}  // namespace numerics
