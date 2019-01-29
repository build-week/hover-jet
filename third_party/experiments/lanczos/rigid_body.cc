#include "lanczos/rigid_body.hh"

#include "eigen_helpers.hh"

namespace lanczos {

RigidBody body_frame(const SE3& body_from_world,
                     const Vec3& body_frame_velocity,
                     const Vec3& body_frame_ang_vel) {
  RigidBody out_body;
  out_body.from_world = body_from_world;

  const VecNd<6> velocity_body_frame =
      jcc::vstack(body_frame_velocity, body_frame_ang_vel);
  const VecNd<6> velocity_world_frame =
      body_from_world.Adj() * velocity_body_frame;

  out_body.vel = velocity_world_frame.head<3>();
  out_body.ang_vel = velocity_world_frame.tail<3>();
  return out_body;
}

RigidBody simulate(const RigidBody& in_body,
                   const RigidBodySimulationConfig& cfg,
                   const double dt_sec) {
  RigidBody out_body;

  //
  // Adjust damping for actual time passed
  //
  const double positional_damping =
      std::pow(cfg.positional_damping_per_sec, dt_sec);
  const double angular_damping = std::pow(cfg.angular_damping_per_sec, dt_sec);

  //
  // Compute new velocities, including damping
  //
  out_body.vel =
      in_body.vel * (1.0 - positional_damping);
  out_body.ang_vel =
      in_body.ang_vel * (1.0 - angular_damping);

  //
  // Compute the change in pose affecting the object
  //
  const VecNd<6> log_delta = jcc::vstack(in_body.vel, in_body.ang_vel) * dt_sec;
  const SE3 out_body_from_in_body = SE3::exp(log_delta);

  out_body.from_world = out_body_from_in_body * in_body.from_world;

  return out_body;
}

}  // namespace lanczos
