#include "geometry/perp.hh"

namespace geometry {

Eigen::VectorXd perp(const Eigen::VectorXd& vec) {
  using Vec = Eigen::VectorXd;

  //
  // Gram-schmidt
  //

  int min_coeff_ind;
  vec.minCoeff(&min_coeff_ind);

  const Vec vec_u = vec.normalized();

  Vec unit = Vec::Zero(vec.size());
  unit[min_coeff_ind] = 1;
  // const Vec unit = Vec::Unit(min_coeff_ind);
  return (unit - (vec_u * unit.dot(vec_u))).normalized();
}

}  // namespace geometry