#pragma once

#include "lanczos/rigid_body.hh"

namespace lanczos {

struct NewtonianObject {
  RigidBody body;
  double mass_kg = 1.0;
  // pkg --> "per kilogram"
  Eigen::Matrix3d inertia_pkg = Eigen::Matrix3d::Identity();

  RigidBodySimulationConfig rigid_body_cfg = {};
};

NewtonianObject simulate(const NewtonianObject& object,
                         const Vec3& force,
                         const Vec3& torque,
                         double dt);
}  // namespace lanczos
