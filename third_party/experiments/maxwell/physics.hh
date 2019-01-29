#pragma once

#include <vector>

#include "eigen.hh"

namespace maxwell {

constexpr double MU_0 = 0.01;
constexpr double SPH_PERM = MU_0 / (4.0 * M_PI);

// All uninitialized, baby
struct WireElement {
  double current;
  Eigen::Vector3d length_blade;
  Eigen::Vector3d location;
};

// An infinitely thin current loop
class ThinCurrentLoop {
 public:
  using Vec3 = Eigen::Vector3d;

  ThinCurrentLoop(const std::vector<WireElement>& elements) : elements_(elements) {}

  // Compute b_field by biot-savart law
  Vec3 b_field(const Vec3& r) const;

  const std::vector<WireElement>& elements() const {
    return elements_;
  }

 private:
  std::vector<WireElement> elements_;
};

ThinCurrentLoop give_me_a_circle(double radius, double current, double y = 0.0);
}
