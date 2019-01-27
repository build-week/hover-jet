#include "lanczos/newtonian_object.hh"

#include "eigen_helpers.hh"

namespace lanczos {

NewtonianObject simulate(const NewtonianObject& object,
                         const Vec3& world_force,
                         const Vec3& world_torque,
                         const double dt_sec) {
  //
  // Generate deltas
  //

  // No inverse, baby!
  // (These solves could feasibly be a bottleneck)
  const Vec3 dv_dt = world_force / object.mass_kg;
  const Vec3 dw_dt = object.inertia_pkg.fullPivLu().solve(world_torque);

  const SE3 body_from_world = object.body.from_world;

  //
  // Integrate
  //

  NewtonianObject new_object = object;

  new_object.body.vel += (dv_dt * dt_sec);
  new_object.body.ang_vel += (dw_dt * dt_sec);

  // v: World Frame
  // w: World Frame; but NOT dT/dt
  const SE3 world_from_body = body_from_world.inverse();

  SE3 new_world_from_body;
  new_world_from_body.translation() =
      new_world_from_body.translation() + (new_object.body.vel * dt_sec);
  new_world_from_body.so3() =
      SO3::exp(new_object.body.ang_vel * dt_sec) * world_from_body.so3();

  new_object.body.from_world = new_world_from_body.inverse();

  return new_object;
}
}  // namespace lanczos