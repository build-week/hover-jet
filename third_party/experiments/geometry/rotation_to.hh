#pragma once

#include "eigen.hh"
#include "sophus.hh"

namespace geometry {

// Lifted from [1]
// [1] https://github.com/jpanikulam/sonder/blob/master/sonder/src/geometry/geometry.cc
SO3 rotation_to(const Eigen::Vector3d &from, const Eigen::Vector3d &to);

}  // namespace geometry