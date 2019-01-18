// %deps(scene_tree)
#include "third_party/experiments/viewer/primitives/scene_tree.hh"

#include "visualization/thrust_stand_visualizer.hh"

namespace jet {
namespace visualization {

namespace {

using control::JetConfiguration;
using control::JetStatus;
using control::QuadraframeConfiguration;
using control::QuadraframeStatus;
using control::VaneConfiguration;

void put_plane(viewer::SimpleGeometry& geo,
               const SE3& world_from_plane,
               const jcc::Vec4& color) {
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
  poly.color = color;

  geo.add_polygon(poly);

  geo.add_axes({world_from_plane, 0.005, 3.0});
}

SE3 vane_unit_from_vane(const double angle, const VaneConfiguration& cfg) {
  const SE3 zero_from_angle = jcc::exp_z(angle);
  return cfg.T_vane_unit_from_vane_default * zero_from_angle;
}
}  // namespace

void setup_view(const std::string& viewer_name) {
  const auto view = viewer::get_window3d(viewer_name);
  view->set_target_from_world(
      SE3(SO3::exp(Eigen::Vector3d(-M_PI * 0.5, 0.0, 0.0)), Eigen::Vector3d::Zero()));
  view->set_continue_time_ms(10);
  view->set_zoom(10.0);

  const auto background = view->add_primitive<viewer::SimpleGeometry>();
  const geometry::shapes::Plane ground{jcc::Vec3::UnitZ(), 0.0};
  background->add_plane({ground});
  background->flip();
}

void put_quadraframe(viewer::SimpleGeometry& geo,
                     const QuadraframeStatus& status,
                     const QuadraframeConfiguration& quad_cfg,
                     const VaneConfiguration& vane_cfg) {
  const jcc::Vec4 normal_color(0.8, 0.8, 0.8, 0.8);
  const jcc::Vec4 vane_0_color(0.8, 0.3, 0.3, 0.8);

  const SE3 com_from_oriented_vane_0 =
      quad_cfg.com_from_vane_unit_0 *
      vane_unit_from_vane(status.servo_0_angle_rad, vane_cfg);
  put_plane(geo, com_from_oriented_vane_0, vane_0_color);

  const SE3 com_from_oriented_vane_1 =
      quad_cfg.com_from_vane_unit_1 *
      vane_unit_from_vane(status.servo_1_angle_rad, vane_cfg);
  put_plane(geo, com_from_oriented_vane_1, normal_color);

  const SE3 com_from_oriented_vane_2 =
      quad_cfg.com_from_vane_unit_2 *
      vane_unit_from_vane(status.servo_2_angle_rad, vane_cfg);
  put_plane(geo, com_from_oriented_vane_2, normal_color);

  const SE3 com_from_oriented_vane_3 =
      quad_cfg.com_from_vane_unit_3 *
      vane_unit_from_vane(status.servo_3_angle_rad, vane_cfg);
  put_plane(geo, com_from_oriented_vane_3, normal_color);
}

}  // namespace visualization
}  // namespace jet
