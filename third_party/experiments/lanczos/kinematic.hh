#pragma once

//%ignore

#include "eigen.hh"
#include "sophus.hh"

#include "lanczos/rigid_body.hh"

namespace lanczos {

template <typename T>
int get_new_key(const std::map<int, T>& mapping) {
  return mapping.rbegin().first() + 1;
}

struct NodePair {
  int first = -1;
  int second = -1;
};

struct Link {
  double mass_kg = 1.0;
  double length_m = 1.0;

  NodePair connects;
};

// [x] Anything can freely join?
// [ ] Two things can join and the joint has angle?
struct BallJoint {
  NodePair connects;

  static constexpr int DOF = 3;
};

struct Node {
  SE3 node_from_world;

  static constexpr int DOF = 6;
};

// Concept
//  - Nodes can be linked with "links" or with "hinges"
//  - Hinges can bend
class KinematicStructure {
  using Vec3 = Eigen::Vector3d;

  int add_link(const Link& link);

  void is_valid();

  // Returns false if it could not be made feasible
  bool make_feasible();

 private:
  std::map<int, Link> links_;
  std::map<int, Node> nodes_;
  std::map<int, BallJoint> joints_;
};

int KinematicStructure::add_node(const Node& node) {
  //
}

int KinematicStructure::add_link(const Link& link) {
  const int new_id = get_new_key(links_);
  return new_id;
}

}  // namespace lanczos
