#pragma once

#include "eigen.hh"
#include "sophus.hh"

namespace lanczos {

namespace {
using Vec3 = Eigen::Vector3d;
}  // namespace
struct RigidBody {
  // NOTE:
  // These properties are specified in the world frame
  Vec3 vel = Vec3::Zero();
  Vec3 ang_vel = Vec3::Zero();
  SE3 from_world;
};

struct RigidBodySimulationConfig {
  double positional_damping_per_sec = 0.0;
  double angular_damping_per_sec = 0.0;
};

RigidBody body_frame(const SE3& body_from_world,
                     const Vec3& body_frame_velocity,
                     const Vec3& body_frame_ang_vel);

RigidBody simulate(const RigidBody& in_body,
                   const RigidBodySimulationConfig& cfg,
                   const double dt);

}  // namespace lanczos