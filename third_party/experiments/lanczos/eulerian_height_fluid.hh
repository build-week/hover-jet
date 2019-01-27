#pragma once

#include "eigen.hh"

#include <array>

namespace lanczos {

class EulerianHeightFluid {
 private:
  using Mat = Eigen::MatrixXd;
  using Vec3 = Eigen::Vector3d;

 public:
  EulerianHeightFluid(const int size, const double scale);

  const Mat height_field() const;

  const Mat& terrain_field() const;

  const Mat& v_x() const;

  const Mat& v_y() const;

  // Update depth
  void update_depth_field(double dt_sec);

  // Advect velocity
  void update_velocity_field(double dt_sec);

  // Brute-enforce boundary conditions
  // (Is applying the constraint via MUM more stable? Accurate?)
  void enforce_boundary_conditions();

  void simulate(double dt_sec);

 private:
  Mat terrain_field_;
  Mat depth_field_;

  std::array<Mat, 2> velocity_field_;
  double scale_mppx_ = 10.0;
};

}  // namespace lanczos
