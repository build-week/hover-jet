
#include "planning/body.hh"
#include "planning/joint_planner.hh"

#include "geometry/intersection/sphere_plane.hh"
#include "geometry/shapes/circle.hh"
#include "geometry/shapes/halfspace.hh"
#include "geometry/shapes/sphere.hh"
#include "geometry/visualization/put_circle.hh"

#include "viewer/primitives/simple_geometry.hh"
#include "viewer/window_3d.hh"

#include <unordered_map>

namespace planning {
using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;

Body make_walker() {
  Body body(SE3(SO3(), Vec3(1.0, 0.0, 1.0)));

  const SE3 joint_from_body_1(SO3::exp(Vec3(0.0, 0.0, M_PI * 0.5)), Vec3(0.1, 0.0, 0.0));
  const SE3 joint_from_body_2(SO3::exp(Vec3(0.0, M_PI * 0.5, 0.0)), Vec3(1.0, 0.0, 0.0));

  int n = body.attach_link(-1, SE3(), {0.0, 0.0, 0.0});
  {
    n = body.attach_link(n, joint_from_body_2, {0.0, 0.7, 0.0});
    n = body.attach_link(n, joint_from_body_2, {0.0, 0.4, 0.0});
    n = body.attach_link(n, joint_from_body_2, {0.0, 0.4, 0.0});
    n = body.attach_link(n, joint_from_body_2, {0.0, 0.4, 0.0});
    n = body.attach_link(n, joint_from_body_1, {0.0, 0.4, 0.0});
  }

  if (0) {
    n = body.attach_link(-1, joint_from_body_2, {0.0, 0.0, 0.3});
    n = body.attach_link(n, joint_from_body_2, {0.0, 0.0, 0.1});
    body.attach_link(n, joint_from_body_2, {0.0, 0.0, 0.0});
  }

  return body;
}

void put_body(viewer::SimpleGeometry& geo,
              const Body& body,
              const Vec4& color = Vec4(1.0, 1.0, 1.0, 1.0)) {
  std::queue<int> q;
  q.emplace(-1);

  std::unordered_map<int, SE3> world_from_joint;
  world_from_joint[-1] = SE3();

  while (!q.empty()) {
    const int parent = q.front();
    q.pop();

    const auto world_from_parent = world_from_joint.at(parent);

    constexpr double JOINT_RADIUS_M = 0.1;
    if (body.parent_to_children().count(parent) == 0) {
      const Vec4 color(1.0, 0.1, 0.1, 0.8);
      geo.add_sphere({world_from_parent.translation(), JOINT_RADIUS_M, color});
      continue;
    } else {
      const Vec4 color(0.1, 1.0, parent == -1 ? 0.8 : 0.1, 0.8);
      geo.add_sphere({world_from_parent.translation(), JOINT_RADIUS_M, color});
    }

    for (const auto& child : body.parent_to_children().at(parent)) {
      const auto& joint = body.joints().at(child.joint);

      const Vec3 log_joint = Vec3(0.0, 0.0, joint.angle);
      const SE3 joint_place_from_joint = SE3(SO3::exp(log_joint), Vec3::Zero());

      world_from_joint[child.joint] =
          world_from_parent * child.parent_from_joint * joint_place_from_joint.inverse();

      geo.add_line({world_from_parent.translation(),
                    world_from_joint[child.joint].translation(), color});

      const geometry::shapes::Circle joint_circle{
          world_from_joint[child.joint].translation(),
          world_from_joint[child.joint].so3() * Vec3::UnitZ(), JOINT_RADIUS_M};
      geometry::visualization::put_circle(geo, joint_circle);

      q.emplace(child.joint);
    }
  }
}

void walk() {
  const auto view = viewer::get_window3d("Mr. Walks, walks");
  view->set_target_from_world(
      SE3(SO3::exp(Eigen::Vector3d(-3.1415 * 0.5, 0.0, 0.0)), Eigen::Vector3d::Zero()));
  const auto geo = view->add_primitive<viewer::SimpleGeometry>();
  const auto plan_geo = view->add_primitive<viewer::SimpleGeometry>();
  view->set_continue_time_ms(20);

  auto walker = make_walker();

  for (int t = 0; t < 1500; ++t) {
    const geometry::shapes::Plane ground{Vec3::UnitZ(), 0.0};
    geo->add_plane({ground});
    geo->add_plane({{Vec3::UnitX(), 0.0}, 1.0,Vec4(1.0, 0.0, 0.0, 0.3)});
    geo->add_plane({{Vec3::UnitY(), 0.0}, 1.0,Vec4(0.0, 1.0, 0.0, 0.3)});

    put_body(*geo, walker);

    const JointPlanner planner(walker);
    const auto planning_problem = planner.generate_opt_funcs();
    const auto opt_problem = planner.build_optimization_problem(planning_problem);
    const VecX plan = planner.optimize(opt_problem);

    constexpr bool draw_plan = false;
    if (draw_plan) {
      const int step_count = 7;
      for (int tt = 0; tt < step_count; ++tt) {
        const auto planned_body = planner.form_body(plan, planning_problem.dynamics, tt);
        const Vec4 line_color(1.0, 0.0, 0.0, static_cast<double>(tt) / step_count);
        put_body(*plan_geo, planned_body, line_color);
      }
    }

    plan_geo->flip();
    walker = planner.form_body(plan, planning_problem.dynamics, 1);

    geo->flip();
    view->spin_until_step();
    if (view->should_close()) {
      break;
    }
  }
}

}  // namespace planning

int main() {
  planning::walk();
}