#pragma once

#include "eigen.hh"
#include "sophus.hh"

#include "planning/body.hh"

#include "numerics/optimize.hh"

namespace planning {

namespace {
using Vec3 = Eigen::Vector3d;
using VecX = Eigen::VectorXd;

}  // namespace

class JointPlanner {
 public:
  JointPlanner(const Body& body) : body_(body) {
  }

  static constexpr int HORIZON = 5;
  static constexpr int U_SIZE = 1;
  static constexpr int X_SIZE = 2;

  using Cost = std::function<double(const VecX&, const VecX&)>;
  using Dynamics = std::function<VecX(const VecX&, const VecX&)>;

  struct PlanningProblem {
    Cost cost;
    Dynamics dynamics;
  };

  //
  // Form the problem
  //

  PlanningProblem generate_opt_funcs() const;

  numerics::OptimizationProblem build_optimization_problem(
      const JointPlanner::PlanningProblem& planning_problem) const;

  VecX optimize(const numerics::OptimizationProblem& problem) const;

  VecX form_state() const;

  // control_sequence: Sequence of controls
  // dyn: Dynamics
  // t: At what time in the plan!
  Body form_body(const VecX& control_sequence, const Dynamics& dyn, int t) const;

  // TODO: Do this in a smarter way (no need for maps, etc)
  Body form_body(const VecX& xt) const;

  //
  // Accessors
  //

  //
  // Indexors
  //

  int angle_ind(const int joint) const {
    return (joint * X_SIZE);
  }
  int velocity_ind(const int joint) const {
    return (joint * X_SIZE) + 1;
  }
  int accel_ind(const int joint) const {
    return joint;
  }

 private:
  Body body_;
};

}  // namespace planning
