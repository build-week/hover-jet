#pragma once

#include "third_party/experiments/eigen.hh"
#include "third_party/experiments/sophus.hh"

namespace jet {
namespace control {

SO3 target_from_joy(double joy_pitch_axis, double joy_roll_axis, double joy_yaw_axis);

}  // namespace control
}  // namespace jet
