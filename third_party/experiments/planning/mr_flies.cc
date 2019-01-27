#include "planning/body.hh"
#include "planning/joint_planner.hh"

#include "geometry/intersection/sphere_plane.hh"
#include "geometry/shapes/circle.hh"
#include "geometry/shapes/halfspace.hh"
#include "geometry/shapes/sphere.hh"
#include "geometry/visualization/put_circle.hh"

#include "lanczos/newtonian_object.hh"

#include "viewer/primitives/simple_geometry.hh"
#include "viewer/window_3d.hh"

#include <unordered_map>

namespace planning {
namespace {
using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;
}  // namespace

using TransferFunction = std::function<double(double)>;

// Thruster:
// Interact by acceleration/decelerating the thing
struct Thruster {
  SE3 body_from_thruster;
  double velocity_mps = 0.0;
  TransferFunction accel_from_current = [](const double i) { return i; };
  TransferFunction thrust_from_vel = [](const double vel) { return vel; };
  double max_thrust_N = 10.0;
  double min_thrust_N = -10.0;
};

struct Wrench {
  Vec3 world_force;
  Vec3 world_torque;
};

class Vehicle {
 public:
  Vehicle() {
    body_.rigid_body_cfg.positional_damping_per_sec = 0.1;
    body_.rigid_body_cfg.angular_damping_per_sec = 0.1;
  }

  void add_thruster(const Thruster& thruster) {
    thrusters_.push_back(thruster);
  }

  Wrench compute_wrench(const std::vector<Thruster>& thrusters) const {
    Vec3 net_body_force = Vec3::Zero();
    Vec3 net_body_torque = Vec3::Zero();
    for (const auto& thruster : thrusters) {
      const double thrust = thruster.thrust_from_vel(thruster.velocity_mps);
      const Vec3 body_force =
          thruster.body_from_thruster.so3() * (Vec3::UnitZ() * thrust);
      net_body_force += body_force;

      const Vec3 r = thruster.body_from_thruster.translation();
      const Vec3 body_torque = r.cross(net_body_force);
      net_body_torque += body_torque;
    }

    net_body_torque += Vec3::UnitX();

    const double g_npkg = 1.0;
    const Vec3 gravity = Vec3::UnitZ() * -g_npkg;
    const Vec3 net_force =
        (body_.body.from_world.so3().inverse() * net_body_force) + gravity;

    const Vec3 wind(0.5, 0.0, 0.0);
    const Vec3 net_torque = (body_.body.from_world.so3().inverse() * net_body_torque);
    return {net_force, net_torque};
  }

  void coarse_simulate(const double dt) {
    const Wrench wrench = compute_wrench(thrusters_);
    body_ = lanczos::simulate(body_, wrench.world_force, wrench.world_torque, dt);
  }

  SE3 from_world() const {
    return body_.body.from_world;
  }

 private:
  lanczos::NewtonianObject body_;
  std::vector<Thruster> thrusters_;
};

void put_vehicle(viewer::SimpleGeometry& geo,
                 const Vehicle& vehicle,
                 const Vec4& color = Vec4(1.0, 1.0, 1.0, 1.0)) {
  const SE3 world_from_body = vehicle.from_world().inverse();
  const geometry::shapes::Circle circle{world_from_body.translation(),
                                        world_from_body.so3() * Vec3::UnitZ(), 0.3};

  geometry::visualization::put_circle(geo, circle);
}

auto setup() {
  const auto view = viewer::get_window3d("Mr. Flies, flies");
  view->set_target_from_world(
      SE3(SO3::exp(Eigen::Vector3d(-3.1415 * 0.5, 0.0, 0.0)), Eigen::Vector3d::Zero()));
  view->set_continue_time_ms(20);

  const auto background = view->add_primitive<viewer::SimpleGeometry>();
  const geometry::shapes::Plane ground{Vec3::UnitZ(), 0.0};
  background->add_plane({ground});
  background->flip();
  return view;
}

void go() {
  const auto view = setup();
  const auto geo = view->add_primitive<viewer::SimpleGeometry>();
  const auto plan_geo = view->add_primitive<viewer::SimpleGeometry>();

  Vehicle quad;

  const double dt = 0.1;
  for (int k = 0; k < 2500; ++k) {
    quad.coarse_simulate(dt);
    put_vehicle(*geo, quad);
    geo->flip();

    view->spin_until_step();
  }
}

}  // namespace planning

int main() {
  planning::go();
}