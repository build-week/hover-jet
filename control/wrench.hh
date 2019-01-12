#pragma once

#include "third_party/experiments/eigen.hh"
#include "third_party/experiments/sophus.hh"

namespace jet {
namespace control {

struct Wrench {
  jcc::Vec3 force_N;
  jcc::Vec3 torque_Nm;
};

Wrench operator+(const Wrench& w1, const Wrench& w2);

// Computes a wrench from a force and moment arm
Wrench wrench_from_force_arm(const jcc::Vec3& force_N, const jcc::Vec3& arm_m);

Wrench wrench_in_frame(const SE3& frame_2_from_frame_1, const Wrench& wrench_frame_1);

void print_wrench(const Wrench& wrench);

}  // namespace control
}  // namespace jet