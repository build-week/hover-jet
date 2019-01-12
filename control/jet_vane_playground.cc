#include "control/jet_vane_model.hh"

// %deps(simple_geometry)
// %deps(scene_tree)
// %deps(window_3d)
#include "third_party/experiments/viewer/primitives/scene_tree.hh"
#include "third_party/experiments/viewer/primitives/simple_geometry.hh"
#include "third_party/experiments/viewer/window_3d.hh"

// %dep(jet_model)
// %dep(environment)
// #include "third_party/experiments/planning/jet/jet_model.hh"

#include "control/quadraframe_model.hh"

namespace jet {
namespace control {

namespace {
void setup() {
  const auto view = viewer::get_window3d("Mr. Vane, vanes");
  view->set_target_from_world(SE3(SO3::exp(Eigen::Vector3d(-M_PI * 0.5, 0.0, 0.0)), Eigen::Vector3d::Zero()));
  view->set_continue_time_ms(10);
  view->set_zoom(10.0);

  const auto background = view->add_primitive<viewer::SimpleGeometry>();
  const geometry::shapes::Plane ground{jcc::Vec3::UnitZ(), 0.0};
  background->add_plane({ground});
  background->flip();
}

void put_plane(viewer::SimpleGeometry& geo, const SE3& world_from_plane) {
  viewer::Polygon poly;

  // The frame here looks like the following
  //
  // (X out of page)
  // ------------------- -Z
  // |
  // |
  // |
  // |
  // |
  // |
  // |
  // +Y

  constexpr double ONE_QUARTER = 1.0 / 4.0;
  constexpr double THREE_QUARTERS = 3.0 / 4.0;

  const jcc::Vec3 vane_tip_edge_from_cp(0.0, 0.0, 10e-3);

  constexpr double vane_x = 20e-3;
  constexpr double vane_z = 30e-3;

  // These are all in the *angled* vane frame
  const std::vector<jcc::Vec3> vane_points_vane_frame = {{
      jcc::Vec3(-vane_x * ONE_QUARTER, 0.0, -vane_z),
      jcc::Vec3(-vane_x * ONE_QUARTER, 0.0, 0.0),
      jcc::Vec3(0.5 * vane_x, 0.0, 0.0),
      jcc::Vec3(vane_x * THREE_QUARTERS, 0.0, -0.2 * vane_z),
      jcc::Vec3(vane_x * THREE_QUARTERS, 0.0, -vane_z),
  }};

  for (const auto& pt_plane_frame : vane_points_vane_frame) {
    poly.points.push_back(world_from_plane * (pt_plane_frame + vane_tip_edge_from_cp));
  }
  poly.outline = true;
  poly.color = jcc::Vec4(0.8, 0.8, 0.8, 0.8);
  geo.add_polygon(poly);

  geo.add_axes({world_from_plane, 0.025, 3.0});
}

SE3 vane_unit_from_vane(const double angle, const VaneConfiguration& cfg) {
  const SE3 zero_from_angle = jcc::exp_z(angle);
  return cfg.T_vane_unit_from_zero * zero_from_angle;
}

void put_quadraframe(viewer::SimpleGeometry& geo,
                     const QuadraframeStatus& status,
                     const QuadraframeConfiguration& quad_cfg,
                     const VaneConfiguration& vane_cfg) {
  const SE3 com_from_oriented_vane_1 =
      quad_cfg.com_from_vane_unit_1 * vane_unit_from_vane(status.servo_1_angle, vane_cfg);
  put_plane(geo, com_from_oriented_vane_1);

  const SE3 com_from_oriented_vane_2 =
      quad_cfg.com_from_vane_unit_2 * vane_unit_from_vane(status.servo_2_angle, vane_cfg);
  put_plane(geo, com_from_oriented_vane_2);

  const SE3 com_from_oriented_vane_3 =
      quad_cfg.com_from_vane_unit_3 * vane_unit_from_vane(status.servo_3_angle, vane_cfg);
  put_plane(geo, com_from_oriented_vane_3);

  const SE3 com_from_oriented_vane_4 =
      quad_cfg.com_from_vane_unit_4 * vane_unit_from_vane(status.servo_4_angle, vane_cfg);
  put_plane(geo, com_from_oriented_vane_4);
}

}  // namespace

void go() {
  setup();
  const auto view = viewer::get_window3d("Mr. Vane, vanes");
  const auto geo = view->add_primitive<viewer::SimpleGeometry>();

  //
  // Generate a force sample
  //

  const JetStatus jet_status({.throttle = 1.0});
  const VaneConfiguration vane_cfg;
  const JetConfiguration jet_cfg;

  const QuadraframeConfiguration quad_cfg = {};

  const double max_angle_rad = 0.2268;
  double change = 0.01;
  double theta_2_rad = -max_angle_rad;
  while (!view->should_close()) {
    theta_2_rad += change;
    if (std::abs(theta_2_rad) > max_angle_rad) {
      change = -change;
      theta_2_rad += change;
    }

    const QuadraframeStatus quad_status = {theta_2_rad, 0.0, 0.0, 0.0};
    put_quadraframe(*geo, quad_status, quad_cfg, vane_cfg);

    const Wrench com_wrench = total_force_com_frame(jet_status, quad_status, vane_cfg, jet_cfg, quad_cfg);

    constexpr double LINE_WIDTH = 4.0;
    const jcc::Vec4 force_color(1.0, 0.5, 0.0, 0.8);
    const jcc::Vec4 torque_color(0.0, 1.0, 1.0, 0.8);
    geo->add_line({jcc::Vec3::Zero(), com_wrench.force_N, force_color, LINE_WIDTH});
    geo->add_line({jcc::Vec3::Zero(), com_wrench.torque_Nm * 10.0, torque_color, LINE_WIDTH * 0.3});

    geo->flip();
    view->spin_until_step();
  }
}

}  // namespace control
}  // namespace jet

int main() {
  jet::control::go();
}