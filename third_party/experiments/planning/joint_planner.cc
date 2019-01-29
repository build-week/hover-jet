#include "planning/joint_planner.hh"

#include "numerics/wrap_optimizer.hh"

namespace planning {

VecX JointPlanner::form_state() const {
  const int n_joints = body_.joints().size();
  VecX x = VecX::Zero(n_joints * X_SIZE);

  for (const auto& joint : body_.joints()) {
    x[angle_ind(joint.first)] = joint.second.angle;
    x[velocity_ind(joint.first)] = joint.second.velocity;
  }
  return x;
}

Body JointPlanner::form_body(const VecX& u,
                             const JointPlanner::Dynamics& dyn,
                             const int t_desired) const {
  Body body = body_;
  const int n_joints = body_.joints().size();

  const VecX x0 = form_state();
  VecX xt = x0;

  for (int t = 0; t < std::min(HORIZON, t_desired); ++t) {
    const VecX ut = u.segment(t, (U_SIZE * n_joints));
    xt = dyn(xt, ut);
  }

  for (const auto& joint : body_.joints()) {
    body.joint(joint.first).angle = xt[angle_ind(joint.first)];
    body.joint(joint.first).velocity = xt[velocity_ind(joint.first)];
  }
  return body;
}

Body JointPlanner::form_body(const VecX& xt) const {
  Body body = body_;
  for (const auto& joint : body_.joints()) {
    body.joint(joint.first).angle = xt[angle_ind(joint.first)];
    body.joint(joint.first).velocity = xt[velocity_ind(joint.first)];
  }
  return body;
}

numerics::OptimizationProblem JointPlanner::build_optimization_problem(
    const JointPlanner::PlanningProblem& planning_problem) const {
  // Forward shooting
  const auto opt_cost = [this, planning_problem](const VecX& u) {
    double total_cost = 0.0;

    const int n_joints = body_.joints().size();

    const VecX x0 = form_state();
    VecX xt = x0;

    for (int t = 0; t < HORIZON; ++t) {
      const VecX ut = u.segment(t, (U_SIZE * n_joints));
      xt = planning_problem.dynamics(xt, ut);
      total_cost += planning_problem.cost(xt, ut);
    }
    return total_cost;
  };

  const auto wrapped_cost = numerics::wrap_numerical_grad(opt_cost);

  const numerics::OptimizationProblem problem{wrapped_cost};
  return problem;
}

VecX JointPlanner::optimize(const numerics::OptimizationProblem& problem) const {
  const int n_joints = body_.joints().size();
  const VecX u_init = VecX::Zero(HORIZON * U_SIZE * n_joints);
  const numerics::OptimizationState initialization{u_init};

  const auto result =
      numerics::optimize<numerics::ObjectiveMethod::kGradientDescent,
                         numerics::ConstraintMethod::kAugLag>(initialization, problem);
  return result.x;
}

JointPlanner::PlanningProblem JointPlanner::generate_opt_funcs() const {
  const Vec3 target(1.0, 1.0, 1.0);
  const int target_joint = 2;

  constexpr double dt = 0.1;
  const auto dynamics = [this, dt](const VecX& xt, const VecX& ut) {
    VecX xtp1 = xt;
    for (const auto& joint : body_.joints()) {
      const int joint_id = joint.first;
      xtp1[angle_ind(joint_id)] += dt * xt[velocity_ind(joint_id)];
      xtp1[velocity_ind(joint_id)] += dt * ut[accel_ind(joint_id)];
    }
    return xtp1;
  };

  const auto per_state_cost = [this, target, target_joint](const VecX& xt,
                                                           const VecX& ut) {
    const auto this_body = form_body(xt);
    const auto root_from_joint = this_body.root_from_joint();

    double cost = 0.0;
    for (const auto& joint : body_.joints()) {
      const int joint_id = joint.first;
      if (joint_id == 3) {
        constexpr int JOINT_TO_GO = 5;
        const Vec3 tx = root_from_joint.at(JOINT_TO_GO).translation();
        const Vec3 error = tx - Vec3(0.0, 1.0, -1.0);

        const double tx_l1 = error.lpNorm<1>();
        const double tx_l2 = error.lpNorm<2>();
        cost += (1.0 * (tx_l1 * tx_l1)) + (50.0 * (tx_l2 * tx_l2));

      } else {
        const double angle = xt[angle_ind(joint_id)];
        const double angle_cost = (angle * angle);
        cost += 0.0 * angle_cost;
      }
      {
        const double vel = xt[velocity_ind(joint_id)];
        const double vel_cost = vel * vel;
        cost += 0.001 * vel_cost;
      }
      {
        const double accel = ut[accel_ind(joint_id)];
        const double ctrl_cost = accel * accel;
        cost += 0.1 * ctrl_cost;
      }
    }
    return cost;
  };

  return {per_state_cost, dynamics};
}

}  // namespace planning

// - Can I come up with a better nonlinear mpc solver?
// - "Tools for synthesizing plausible motions"