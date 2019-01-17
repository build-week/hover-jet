#pragma once

#include "third_party/experiments/eigen.hh"
#include "third_party/experiments/sophus.hh"

#include "control/wrench_generated.hh"

#include <string>

namespace jet {
namespace control {

// Computes a wrench from a force and moment arm
Wrench wrench_from_force_arm(const jcc::Vec3& force_N, const jcc::Vec3& arm_m);

Wrench wrench_in_frame(const SE3& frame_2_from_frame_1, const Wrench& wrench_frame_1);

std::string print_wrench(const Wrench& wrench);

Wrench operator+(const Wrench& wrench_a, const Wrench& wrench_b);
Wrench operator-(const Wrench& wrench_a, const Wrench& wrench_b);

}  // namespace control
}  // namespace jet
