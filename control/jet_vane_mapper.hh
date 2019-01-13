#pragma once

#include "control/quadraframe_model.hh"
#include "control/vanes.hh"
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

  MappingResult map_wrench(const Wrench& target_wrench, const JetStatus& current_jet_status) const;

 private:
  VaneConfiguration vane_cfg_;
  JetConfiguration jet_cfg_;
  QuadraframeConfiguration qframe_cfg_;
};

}  // namespace control
}  // namespace jet