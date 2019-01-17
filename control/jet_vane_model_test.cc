#include "control/jet_vane_model.hh"

#include <cassert>

namespace jet {
namespace control {

// TODO: Setup gtest

void test_force_monotonic_in_throttle() {
  //
  // Setup
  //

  VaneConfiguration vane_cfg;
  vane_cfg.T_vane_unit_from_default = SE3();

  const JetConfiguration jet_cfg;
  const double commanded_servo_angle_rad = 0.0;

  const JetStatus status_zero({.throttle = 0.0});
  const JetStatus status_half({.throttle = 0.5});
  const JetStatus status_full({.throttle = 1.0});

  //
  // Action
  //

  const Wrench wrench_zero_throttle =
      vane_unit_wrench(commanded_servo_angle_rad, status_zero, vane_cfg, jet_cfg);
  const Wrench wrench_half_throttle =
      vane_unit_wrench(commanded_servo_angle_rad, status_half, vane_cfg, jet_cfg);
  const Wrench wrench_full_throttle =
      vane_unit_wrench(commanded_servo_angle_rad, status_full, vane_cfg, jet_cfg);

  //
  // Verification
  //

  const double drag_zero = wrench_zero_throttle.force_N[0];
  const double drag_half = wrench_half_throttle.force_N[0];
  const double drag_full = wrench_full_throttle.force_N[0];

  const double lift_zero = wrench_zero_throttle.force_N[1];
  const double lift_half = wrench_half_throttle.force_N[1];
  const double lift_full = wrench_full_throttle.force_N[1];

  // These numbers would not be zero in a nonzero swirl configuration
  assert(drag_zero == 0.0);
  assert(drag_half > 0.0);
  assert(drag_full > 0.0);

  assert(drag_zero < drag_half);
  assert(drag_half < drag_full);

  assert(lift_zero == 0.0);
  assert(lift_half == 0.0);
  assert(lift_full == 0.0);
}

void test_force_monotonic_in_throttle_finite_angle_of_attack() {
  //
  // Setup
  //

  VaneConfiguration vane_cfg;
  vane_cfg.T_vane_unit_from_default = SE3();

  const JetConfiguration jet_cfg;
  // (This corresponds to a positive angle of attack)
  const double commanded_servo_angle_rad = -0.1;

  const JetStatus status_zero({.throttle = 0.0});
  const JetStatus status_half({.throttle = 0.5});
  const JetStatus status_full({.throttle = 1.0});

  //
  // Action
  //

  const Wrench wrench_zero_throttle =
      vane_unit_wrench(commanded_servo_angle_rad, status_zero, vane_cfg, jet_cfg);
  const Wrench wrench_half_throttle =
      vane_unit_wrench(commanded_servo_angle_rad, status_half, vane_cfg, jet_cfg);
  const Wrench wrench_full_throttle =
      vane_unit_wrench(commanded_servo_angle_rad, status_full, vane_cfg, jet_cfg);

  //
  // Verification
  //

  const double drag_zero = wrench_zero_throttle.force_N[0];
  const double drag_half = wrench_half_throttle.force_N[0];
  const double drag_full = wrench_full_throttle.force_N[0];

  const double lift_zero = wrench_zero_throttle.force_N[1];
  const double lift_half = wrench_half_throttle.force_N[1];
  const double lift_full = wrench_full_throttle.force_N[1];

  constexpr double FEPS = 1e-9;
  assert(wrench_zero_throttle.torque_Nm.norm() < FEPS);
  assert(wrench_half_throttle.torque_Nm.norm() < FEPS);
  assert(wrench_full_throttle.torque_Nm.norm() < FEPS);

  // These numbers would not be zero in a nonzero swirl configuration
  assert(drag_zero == 0.0);
  assert(drag_half > 0.0);
  assert(drag_full > 0.0);

  assert(drag_zero < drag_half);
  assert(drag_half < drag_full);

  assert(lift_zero == 0.0);
  assert(lift_half > 0.0);
  assert(lift_full > 0.0);
  assert(lift_half > lift_zero);
  assert(lift_full > lift_half);
}

void test_torque_non_identity_offset() {
  //
  // Setup
  //

  VaneConfiguration vane_cfg;
  vane_cfg.T_vane_unit_from_default = SE3(SO3(), jcc::Vec3(0.0, 0.0, -1.0));

  const JetConfiguration jet_cfg;
  // (This corresponds to a positive angle of attack)
  const double commanded_servo_angle_rad = 0.0;

  const JetStatus status_full({.throttle = 1.0});

  //
  // Action
  //

  const Wrench wrench_full_throttle =
      vane_unit_wrench(commanded_servo_angle_rad, status_full, vane_cfg, jet_cfg);

  //
  // Verification
  //

  constexpr double FEPS = 1e-9;
  assert(std::abs(wrench_full_throttle.torque_Nm[0]) < FEPS);
  assert(wrench_full_throttle.torque_Nm[1] < 0.0);
  assert(std::abs(wrench_full_throttle.torque_Nm[2]) < FEPS);
}

void run_test() {
  test_force_monotonic_in_throttle();
  std::cout << "test_force_monotonic_in_throttle: ok" << std::endl;
  test_force_monotonic_in_throttle_finite_angle_of_attack();
  std::cout << "test_force_monotonic_in_throttle_finite_angle_of_attack: ok" << std::endl;
  test_torque_non_identity_offset();
  std::cout << "test_torque_non_identity_offset: ok" << std::endl;
}

}  // namespace control
}  // namespace jet

int main() {
  jet::control::run_test();
}
