#include "maxwell/physics.hh"

namespace maxwell {

// https://en.wikipedia.org/wiki/Biot%E2%80%93Savart_law#Electric_currents_.28along_closed_curve.29
ThinCurrentLoop::Vec3 ThinCurrentLoop::b_field(const Vec3& r) const {
  Vec3 B = Vec3::Zero();
  for (std::size_t k = 0; k < elements_.size(); ++k) {
    const auto& el = elements_[k];
    const Vec3 r_prime = r - el.location;
    const double r_prime_norm = r_prime.norm();

    const double inv_r_prime_norm_cbd = 1.0 / (r_prime_norm * r_prime_norm * r_prime_norm);

    // Contribute to the integral
    B += SPH_PERM * (el.current * el.length_blade.cross(r_prime)) * inv_r_prime_norm_cbd;
  }
  return B;
}

ThinCurrentLoop give_me_a_circle(double radius, double current, double y) {
  std::vector<WireElement> elements;
  constexpr double INTERVAL = 0.01;

  const Eigen::Vector3d normal = Eigen::Vector3d::UnitY();
  const Eigen::Vector3d center = Eigen::Vector3d(0.0, y, 0.0);

  for (double w = 0.0; w < 2.0 * M_PI; w += INTERVAL) {
    WireElement element;
    const double x = radius * std::sin(w);
    const double z = radius * std::cos(w);
    element.location = Eigen::Vector3d(x, y, z);

    element.current = current;
    const Eigen::Vector3d r = element.location - center;
    element.length_blade = INTERVAL * radius * normal.cross(r).normalized();
    elements.push_back(element);
  }

  return ThinCurrentLoop(elements);
}
}
