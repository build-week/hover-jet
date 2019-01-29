
#include "eigen.hh"
#include "eigen_helpers.hh"

#include "numerics/numdiff.hh"
#include "sophus/se3.hpp"

using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using Vec6 = VecNd<6>;

using Scalar = double;
using SE3    = Sophus::SE3<Scalar>;
using SO3    = Sophus::SO3<Scalar>;

int main() {
  const SE3  T      = SE3(SO3::exp(Vec3(-0.2, 0.1, 0.3)), Vec3(0.0, 0.0, 1.0));
  const Vec3 obj_pt = Vec3::Random();

  const MatNd<3, 3> K = MatNd<3, 3>::Random();

  const auto error_fcn = [&T, &obj_pt, &K](const Vec6& log_camera_from_object) {
    return (K * (SE3::exp(log_camera_from_object) * (T * obj_pt))).eval();
  };

  const MatNd<3, 6> J_k = numerics::numerical_jacobian<3, 6>(Vec6::Zero(), error_fcn, 1e-6);
  std::cout << J_k << std::endl;

  const MatNd<3, 6> analytical_jac_se3 = jcc::hstack(MatNd<3, 3>::Identity().eval(), SO3::hat(T * obj_pt));

  std::cout << K * analytical_jac_se3 << std::endl;
}
