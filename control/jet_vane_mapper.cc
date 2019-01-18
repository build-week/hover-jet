#include "control/jet_vane_mapper.hh"

#include "third_party/experiments/numerics/group_diff.hh"

namespace jet {
namespace control {

MappingResult JetVaneMapper::map_wrench(const Wrench& target_wrench, const JetStatus& current_jet_status) const {
  using InformationMat = MatNd<QuadraframeStatus::DIM, QuadraframeStatus::DIM>;
  using JacobianMat = MatNd<Wrench::DIM, QuadraframeStatus::DIM>;

  // Wrench Error Jacobian
  const auto held_function = [this, &current_jet_status](const QuadraframeStatus& qframe_status) -> Wrench {
    return total_wrench_com_frame(current_jet_status, qframe_status, vane_cfg_, jet_cfg_, qframe_cfg_);
  };

  //
  // Minimize
  //

  // Small levenberg damping to avoid numerical issues or unboundedness
  constexpr double LEVENBERG_COEFF = 1e-3;

  QuadraframeStatus cur_qframe_stat;
  constexpr int MAX_ITERS = 5;
  for (int k = 0; k < MAX_ITERS; ++k) {
    const Wrench got_wrench = held_function(cur_qframe_stat);
    const VecNd<Wrench::DIM> residual = Wrench::compute_delta(target_wrench, got_wrench);

    const JacobianMat J = numerics::group_jacobian<QuadraframeStatus, Wrench>(cur_qframe_stat, held_function);
    const InformationMat JtJ = (LEVENBERG_COEFF * InformationMat::Identity()) + (J.transpose() * J);

    const Eigen::LLT<InformationMat> JtJ_llt(JtJ);
    const VecNd<QuadraframeStatus::DIM> qframe_delta = JtJ_llt.solve(J.transpose() * residual);

    cur_qframe_stat = QuadraframeStatus::apply_delta(cur_qframe_stat, qframe_delta);
  }

  //
  // Return
  //

  const MappingResult result{.optimal_status = cur_qframe_stat, .achieved_wrench = held_function(cur_qframe_stat)};

  return result;
}

}  // namespace control
}  // namespace jet