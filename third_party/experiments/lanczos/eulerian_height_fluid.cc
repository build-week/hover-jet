#include "lanczos/eulerian_height_fluid.hh"

#include "fluids/fields_2d.hh"

namespace fluids = jcc::fluids::plane;

namespace lanczos {

using Mat = Eigen::MatrixXd;
using Vec3 = Eigen::Vector3d;

EulerianHeightFluid::EulerianHeightFluid(const int size, const double scale) {
  depth_field_ = Mat::Ones(size, size) * 25.0;
  terrain_field_ = Mat::Ones(size, size) * 1.0;
  velocity_field_[0] = Mat::Zero(size, size);
  velocity_field_[1] = Mat::Zero(size, size);

  scale_mppx_ = scale;

  for (int x = 0; x < size; ++x) {
    for (int y = 0; y < size; ++y) {
      terrain_field_(x, y) = 1.0;
      depth_field_(x, y) = 23.0;
    }
  }

  for (int x = 190; x < 200; ++x) {
    for (int y = 190; y < 200; ++y) {
      velocity_field_[0](x, y) += 0.6;
      velocity_field_[1](x, y) += 0.6;
    }
  }

  for (int x = 210; x < 250; ++x) {
    for (int y = 210; y < 250; ++y) {
      terrain_field_(x, y) = 28.0;
      depth_field_(x, y) = 0.0;
    }
  }
}

const Mat EulerianHeightFluid::height_field() const {
  return depth_field_ + terrain_field_;
}

const Mat& EulerianHeightFluid::terrain_field() const {
  return terrain_field_;
}

const Mat& EulerianHeightFluid::v_x() const {
  return velocity_field_[0];
}

const Mat& EulerianHeightFluid::v_y() const {
  return velocity_field_[1];
}

// Update depth
void EulerianHeightFluid::update_depth_field(double dt_sec) {
  const int size = depth_field_.cols();
  const double inv_scale = 1.0 / scale_mppx_;

  const auto& u = velocity_field_;
  const auto& d = depth_field_;

  const Mat dvx =
      (d.topRows(size - 1).array() * u[0].topRows(size - 1).array()) -
      (d.bottomRows(size - 1).array() * u[0].bottomRows(size - 1).array());

  const Mat dvy =
      (d.leftCols(size - 1).array() * u[1].leftCols(size - 1).array()) -
      (d.rightCols(size - 1).array() * u[1].rightCols(size - 1).array());

  const Mat delta_depth_field = (dvx.block(0, 1, size - 1, size - 1) +
                                 dvy.block(1, 0, size - 1, size - 1)) *
                                inv_scale;

  depth_field_.block(1, 1, size - 1, size - 1) += -delta_depth_field * dt_sec;
  depth_field_ = depth_field_.array().max(0.0);
  depth_field_ = depth_field_.array().min(50.0);
}

// Advect velocity
void EulerianHeightFluid::update_velocity_field(double dt_sec) {
  const int size = depth_field_.cols();
  const double inv_scale = 1.0 / scale_mppx_;

  const auto h = depth_field_ + terrain_field_;
  const double g_inv_dx = -1.0 * inv_scale;

  velocity_field_[0].block(0, 0, size - 1, size) +=
      g_inv_dx * (h.topRows(size - 1) - h.bottomRows(size - 1)) * dt_sec;

  velocity_field_[1].block(0, 0, size, size - 1) +=
      g_inv_dx * (h.leftCols(size - 1) - h.rightCols(size - 1)) * dt_sec;

  const double damping = 1e-6;
  const double damping_factor = 1.0 - damping;
  velocity_field_ = fluids::mul(velocity_field_, damping_factor);
}

// Brute-enforce boundary conditions
// (Is applying the constraint via MUM more stable? Accurate?)
void EulerianHeightFluid::enforce_boundary_conditions() {
  for (int k = 0; k < 2; ++k) {
    velocity_field_[k].topRows(1) *= 0.0;
    velocity_field_[k].bottomRows(1) *= 0.0;
    velocity_field_[k].leftCols(1) *= 0.0;
    velocity_field_[k].rightCols(1) *= 0.0;
  }

  for (int x = 209; x < 251; ++x) {
    for (int y = 209; y < 251; ++y) {
      // terrain_field_(x, y) = 28.0;
      // depth_field_(x, y) = 0.0;
      velocity_field_[0](x, y) *= 0.1;
      velocity_field_[1](x, y) *= 0.1;
    }
  }
}

void EulerianHeightFluid::simulate(double dt_sec) {
  //
  // Update the depth field
  //

  update_depth_field(dt_sec);

  //
  // Update the velocity field
  //

  update_velocity_field(dt_sec);

  //
  // Enforce boundary conditions
  //

  enforce_boundary_conditions();
}

}  // namespace lanczos