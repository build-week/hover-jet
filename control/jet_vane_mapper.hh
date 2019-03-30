#pragma once

#include "control/quadraframe_model.hh"
#include "control/vanes_generated.hh"
#include "control/wrench.hh"

namespace jet {
namespace control {

struct MappingResult {
  QuadraframeStatus optimal_status;
  Wrench achieved_wrench;
};

// This is class because, in the future, we expect to update the parameters as it runs,
// or otherwise interact with in more ways than a single method
class JetVaneMapper {
 public:
  JetVaneMapper() = default;

  // target_wrench: The desired wrench
  // current_jet_status: The status of the jet (including throttle)
  // relative_force_weight: The weighting for achievement of *force* vs. achievement of torque
  //                        - For exactly feasible wrenches, this will generally have no effect
  //                        - For infeasible wrenches, this will bias towards matching torque instead of force
  MappingResult map_wrench(const Wrench& target_wrench,
                           const JetStatus& current_jet_status,
                           const double relative_force_weight) const;

  // qframe_status: The current status of the quadraframe assembly
  // jet_status: The current status of the engine itself
  Wrench wrench_for_status(const QuadraframeStatus& qframe_status, const JetStatus& jet_status) const;

 private:
  VaneConfiguration vane_cfg_;
  JetConfiguration jet_cfg_;
  QuadraframeConfiguration qframe_cfg_;
};

}  // namespace control
}  // namespace jet
