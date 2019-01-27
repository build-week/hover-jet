#include "viewer/interaction/view3d.hh"

namespace viewer {

SE3 OrbitCamera::camera_from_anchor() const {
  const SO3 az_rot = SO3::exp(Vec3::UnitY() * azimuth_);
  const SO3 elev_rot = SO3::exp(Vec3::UnitX() * elevation_);
  const SO3 elev_az = elev_rot * az_rot;

  const SE3 instantaneous_rotation(SO3(elev_az), Vec3::Zero());
  const SE3 offset(SE3(SO3(), Vec3(0.0, 0.0, -1.0)));
  return offset * instantaneous_rotation;
}

SE3 OrbitCamera::camera_from_world() const {
  return camera_from_anchor() * anchor_body_.from_world;
}

SE3 OrbitCamera::anchor_from_world() const {
  return anchor_body_.from_world;
}

double OrbitCamera::zoom() const {
  return zoom_;
}

OrbitCamera OrbitCamera::simulate(const double dt_sec) const {
  // Copy ourselves into the output camera
  OrbitCamera out_camera = *this;

  const lanczos::RigidBodySimulationConfig cfg(
      {.positional_damping_per_sec = 1e-9, .angular_damping_per_sec = 1e-9});

  out_camera.anchor_body_ = lanczos::simulate(anchor_body_, cfg, dt_sec);
  return out_camera;
}

void OrbitCamera::apply_keys(const KeyMap& keys, double dt_sec) {
  constexpr double acceleration_mpss = 0.005;
  const double delta_v_mps = dt_sec * acceleration_mpss;

  Vec3 delta_vel = Vec3::Zero();
  for (const auto& key_element : keys) {
    const bool held = key_element.second;
    const int key = key_element.first;

    if (!held) {
      continue;
    }

    switch (key) {
      case (static_cast<int>('W')):
        delta_vel(2) += delta_v_mps;
        break;

      case (static_cast<int>('A')):
        delta_vel(0) += delta_v_mps;
        break;

      case (static_cast<int>('S')):
        delta_vel(2) -= delta_v_mps;

        break;
      case (static_cast<int>('D')):
        delta_vel(0) -= delta_v_mps;
        break;

      case (static_cast<int>('C')):
        delta_vel(1) += delta_v_mps;
        break;

      case (static_cast<int>('Z')):
        delta_vel(1) -= delta_v_mps;
        break;
    }
  }

  anchor_body_.vel -= delta_vel;
}

void OrbitCamera::apply_mouse(const WindowPoint& mouse_pos,
                              const WindowPoint& mouse_prev,
                              const bool left_mouse_held,
                              const bool right_mouse_held) {
  const Vec2 motion = mouse_pos.point - mouse_prev.point;

  if (left_mouse_held) {
    azimuth_ += motion(0) * 0.005;
    elevation_ += motion(1) * 0.005;

    if (azimuth_ > M_PI) {
      azimuth_ = -M_PI;
    } else if (azimuth_ < -M_PI) {
      azimuth_ = M_PI;
    }

    if (elevation_ > M_PI_2) {
      elevation_ = M_PI_2;
    } else if (elevation_ < -M_PI_2) {
      elevation_ = -M_PI_2;
    }
  }

  if (right_mouse_held) {
    const Vec3 motion_camera_frame(motion.x(), -motion.y(), 0.0);

    const double inv_zoom = 1.0 / zoom_;

    const Vec3 motion_anchor_frame =
        camera_from_anchor().so3().inverse() * (motion_camera_frame * inv_zoom * 0.05);

    anchor_body_.vel = 5.0 * motion_anchor_frame;
  }
}

void OrbitCamera::apply_scroll(const double amount) {
  zoom_ *= std::exp(0.1 * amount);
  if (zoom_ <= 0.0001) {
    zoom_ = 0.0001;
  }
}

}  // namespace viewer