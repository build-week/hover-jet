#include "planning/body.hh"

namespace planning {

using Vec3 = Eigen::Vector3d;

Body::Body(const SE3& root_from_world) {
  root_from_world_ = root_from_world;
}

//
// Body Manipulation
//

int Body::attach_link(const int parent,
                      const SE3& child_from_parent,
                      const Joint& joint) {
  joints_[current_id_] = joint;
  parent_to_children_[parent].push_back({current_id_, child_from_parent});

  return current_id_++;
}

//
// Simulation
//

// Simulate, purely kinematic
void Body::coarse_simulate(const double dt) {
  for (auto& joint : joints_) {
    joint.second.angle += joint.second.velocity * dt;
    joint.second.velocity *= std::pow(1.0 - joint.second.damping, dt);
  }
}

std::unordered_map<int, SE3> Body::root_from_joint() const {
  std::queue<int> q;
  q.emplace(0);

  std::unordered_map<int, SE3> root_from_joint;
  root_from_joint[0] = SE3();

  while (!q.empty()) {
    const int parent = q.front();
    q.pop();

    const auto world_from_parent = root_from_joint.at(parent);
    if (parent_to_children_.count(parent) == 0) {
      continue;
    }

    for (const auto& child : parent_to_children_.at(parent)) {
      const auto& joint = joints_.at(child.joint);

      //
      // TODO(jake): This is broken --
      //  just don't use end_from_joint when establishing parent_from_joint
      //  can be fixed by being a bit smarter
      //

      // This is a little tricky -- is it right?
      const SO3 end_from_joint_rot = SO3::exp(Vec3(0.0, 0.0, joint.angle));
      const SE3 end_from_joint = SE3(end_from_joint_rot, Vec3::Zero());

      const auto parent_from_joint = child.parent_from_joint * end_from_joint.inverse();
      root_from_joint[child.joint] = world_from_parent * parent_from_joint;

      q.emplace(child.joint);
    }
  }
  return root_from_joint;
}

//
// Accessors
//

const std::unordered_map<int, Joint>& Body::joints() const {
  return joints_;
}
const Body::LinkGraph& Body::parent_to_children() const {
  return parent_to_children_;
}

SE3 Body::root_from_world() const {
  return root_from_world_;
}

}  // namespace planning