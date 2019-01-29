#include "geometry/rotation_to.hh"

namespace geometry {

SO3 rotation_to(const Eigen::Vector3d &from, const Eigen::Vector3d &to) {
  // Enforce normalization when computing the axis
  const Eigen::Vector3d u_from = from.normalized();
  const Eigen::Vector3d u_to = to.normalized();

  const Eigen::Vector3d u_axis = u_from.cross(u_to).normalized();
  const float angle = std::acos(u_from.dot(u_to));

  const Eigen::Vector3d angle_axis = u_axis * angle;
  return SO3::exp(angle_axis);
}
}  // namespace geometry
