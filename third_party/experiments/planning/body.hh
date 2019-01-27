#pragma once

#include "eigen.hh"
#include "sophus.hh"

#include <queue>
#include <unordered_map>
#include <utility>

namespace planning {

struct Joint {
  double damping = 0.0;
  double angle = 0.0;
  double velocity = 0.0;
};

struct Link {
  int joint;
  SE3 parent_from_joint;
};

// Compute force at each constrained location by estimating lagrange multiplier
//
// TODO:
// - ? Make all joints rotate -- so invert everything
//    - But that would not be ideal for a few reasons
//
// - Give every joint a "torque" capability
//   - This way, we can apply the force of gravity
//
class Body {
 public:
  using LinkGraph = std::unordered_map<int, std::vector<Link>>;
  Body(const SE3& root_from_world);

  //
  // Body Manipulation
  //

  int attach_link(const int parent, const SE3& child_from_parent, const Joint& joint);

  //
  // Simulation
  //

  // Simulate, purely kinematic
  void coarse_simulate(const double dt);

  // Compute root_from_joint for all joints
  std::unordered_map<int, SE3> root_from_joint() const;

  //
  // Accessors
  //

  const std::unordered_map<int, Joint>& joints() const;
  Joint& joint(const int joint_ind) {
    return joints_[joint_ind];
  }

  const LinkGraph& parent_to_children() const;

  SE3 root_from_world() const;

 private:
  std::unordered_map<int, Joint> joints_;
  LinkGraph parent_to_children_;
  int current_id_ = 0;

  SE3 root_from_world_;
};

}  // namespace planning