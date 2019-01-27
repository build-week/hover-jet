//%ignore
#include "maxwell/physics.hh"

#include "viewer/window.hh"
#include "viewer/primitives/plot.hh"

#include <memory>
#include <iostream>

int main() {
  using Vec2 = Eigen::Vector2d;
  using Vec3 = Eigen::Vector3d;

  // Generate the loops
  std::vector<maxwell::ThinCurrentLoop> loops;
  for (double cy = -1.0; cy < 1.0; cy += 0.25) {
    constexpr double CURRENT_AMPS = 10.0;
    constexpr double RADIUS_M = 1.0;
    const auto circle = maxwell::give_me_a_circle(RADIUS_M, CURRENT_AMPS, cy);
    loops.push_back(circle);
  }

  auto viewer = viewer::get_window2d("Current Loop B-Field");

  std::cout << "Generating field..." << std::endl;
  constexpr double INTERVAL = 0.1;
  const double z = 0.0;
  for (double x = -5.0; x <= 5.0; x += INTERVAL) {
    for (double y = -5.0; y <= 5.0; y += INTERVAL) {
      Vec3 net_b = Vec3::Zero();
      const Vec3 r = Vec3(x, y, z);
      // This is definitely not locality optimal. Whatever.
      for (const auto& loop : loops) {
        const Vec3 b = loop.b_field(r);
        net_b += b;
      }

      const Vec2 r_2d = Vec2(r.x(), r.y());
      const Vec2 b_2d = Vec2(net_b.x(), net_b.y());
      viewer->add_line({r_2d, r_2d + b_2d});
    }
  }

  //
  // Magnetic Gradient Field
  //
  /*  constexpr double INTERVAL = 0.1;
    const double z = 0.0;
    for (double x = -5.0; x <= 5.0; x += INTERVAL) {
      for (double y = -5.0; y <= 5.0; y += INTERVAL) {
        const Vec3 r = Vec3(x, y, z);
        // This is definitely not locality optimal. Whatever.
        Vec3 net_b = Vec3::Zero();
        for (const auto& loop : loops) {
          const Vec3 b1 = loop.b_field(r);
          const Vec3 b2 = loop.b_field(r + Vec3::UnitY() * 0.01);
          net_b += (b1 - b2) / 0.01;
        }

        const Vec2 r_2d = Vec2(r.x(), r.y());
        const Vec2 b_2d = Vec2(net_b.x(), net_b.y());
        viewer->add_line({r_2d, r_2d + b_2d});
      }
    }*/

  std::cout << "Done generating field" << std::endl;
  viewer::WindowManager::spin();
}
