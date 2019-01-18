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

jcc::Vec4 red() {
  return jcc::Vec4(1.0, 0.0, 0.0, 0.8);
}
jcc::Vec4 green() {
  return jcc::Vec4(0.0, 1.0, 0.0, 0.8);
}
jcc::Vec4 blue() {
  return jcc::Vec4(0.0, 0.0, 1.0, 0.8);
}
jcc::Vec4 white() {
  return jcc::Vec4(1.0, 1.0, 1.0, 0.8);
}

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
  // view->set_target_from_world(
  // SE3(SO3::exp(Eigen::Vector3d(-M_PI * 0.5, 0.0, 0.0)), Eigen::Vector3d::Zero()));

  view->set_target_from_world(
      SE3(SO3::exp(Eigen::Vector3d(0.0, 0.0, M_PI)), Eigen::Vector3d::Zero()));

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

struct ThrustStandModel {
  // The sensor-id's (from the arduino serial output) are
  // not the same as the sensor names
  std::map<int, std::string> sensor_name_from_id = {
      {3, "Z-0"},  //
      {5, "Z-1"},  //
      {0, "X-0"},  //
      {4, "X-1"},  //
      {1, "Y-0"},  //
      {2, "Y-1"},  //
  };

  // Ballpark locations of sensors, meters
  // TODO(jake): Load these from CAD
  std::map<std::string, jcc::Vec3> locations_stand_frame_m = {
      {"Z-0", 0.0254 * jcc::Vec3(-12.75, 0.0, -4.0)},
      {"Z-1", 0.0254 * jcc::Vec3(-5.9, 0.0, -4.0)},
      {"X-0", 0.0254 * jcc::Vec3(-5.0, -2.75, -20.5)},
      {"X-1", 0.0254 * jcc::Vec3(-5.0, 2.75, -20.5)},
      {"Y-0", 0.0254 * jcc::Vec3(-7.875, 4.00, -18.25)},
      {"Y-1", 0.0254 * jcc::Vec3(-7.875, 4.00, -5.25)}};

  // The direction the load cell is "pointing" such that when force is
  // applied in its direction
  std::map<std::string, jcc::Vec3> directions_stand_frame_m = {
      // X and Y sensors measure *positive* when compressed
      {"X-0", 0.0254 * -jcc::Vec3::UnitX()},  //
      {"X-1", 0.0254 * -jcc::Vec3::UnitX()},  //
      {"Y-0", 0.0254 * -jcc::Vec3::UnitY()},  //
      {"Y-1", 0.0254 * -jcc::Vec3::UnitY()},  //

      // The Z sensors appear to measure the opposite sign for compression vs tension
      {"Z-0", 0.0254 * jcc::Vec3::UnitZ()},  //
      {"Z-1", 0.0254 * jcc::Vec3::UnitZ()}   //
  };
};

void put_thrust_stand(viewer::SimpleGeometry& geo,
                      const ThrustStandStatus& stand_status) {
  ThrustStandModel stand_model;

  // const SE3 SECOND_FROM_FIRST = jcc::exp_z(M_PI * 0.5);

  // const SE3 THIRD_FROM_SECOND = jcc::exp_y(M_PI);

  // const SE3 quadraframe_from_stand = THIRD_FROM_SECOND * SECOND_FROM_FIRST;
  const SE3 quadraframe_from_stand(SO3(), jcc::Vec3(0.2, 0.0, 0.5));

  geo.add_line({quadraframe_from_stand * stand_model.locations_stand_frame_m.at("X-0"),
                quadraframe_from_stand * stand_model.locations_stand_frame_m.at("X-1"),
                red(), 5.0});
  geo.add_line({quadraframe_from_stand * stand_model.locations_stand_frame_m.at("Y-0"),
                quadraframe_from_stand * stand_model.locations_stand_frame_m.at("Y-1"),
                green(), 5.0});
  geo.add_line({quadraframe_from_stand * stand_model.locations_stand_frame_m.at("Z-0"),
                quadraframe_from_stand * stand_model.locations_stand_frame_m.at("Z-1"),
                blue(), 5.0});

  for (const auto& load_cell_from_id : stand_status.load_cell_value_from_id) {
    const int id = load_cell_from_id.first;
    const double value = load_cell_from_id.second;
    const std::string load_cell_name = stand_model.sensor_name_from_id.at(id);

    const jcc::Vec3 position =
        quadraframe_from_stand * stand_model.locations_stand_frame_m.at(load_cell_name);
    const jcc::Vec3 direction = quadraframe_from_stand.so3() *
                                stand_model.directions_stand_frame_m.at(load_cell_name);

    constexpr double SCALING = 0.00001;
    const jcc::Vec4 color_negative(1.0, 1.0, 0.0, 0.8);
    const jcc::Vec4 color_positive(0.0, 1.0, 1.0, 0.8);

    const jcc::Vec4 color = value > 0.0 ? color_positive : color_negative;

    geo.add_line({position, position + (direction * value * SCALING), color, 3.0});
  }
}

}  // namespace visualization
}  // namespace jet
