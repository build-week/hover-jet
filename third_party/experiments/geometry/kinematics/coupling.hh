#pragma once

#include "sophus.hh"
#include "eigen.hh"

namespace geometry {
namespace kinematics {

jcc::Vec6 observed_posedot(const SE3& mover_from_observer, const jcc::Vec6& posedot);

}  // namespace kinematics
}  // namespace geometry
