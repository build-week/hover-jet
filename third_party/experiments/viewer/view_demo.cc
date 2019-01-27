#include "viewer/window.hh"
#include "viewer/window_2d.hh"
#include "viewer/window_manager.hh"

#include "viewer/primitives/plot.hh"

#include <iostream>
#include <memory>
#include <thread>

using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;

namespace viewer {

void run() {
  auto win2d_1 = get_window2d("Window A");
  auto win2d_2 = get_window2d("Window B");

  const Vec2 v(0.0, 1.0);
  win2d_1->add_line({Vec2(-0.5, -0.5), Vec2(0.5, 0.5)});
  win2d_1->add_line({Vec2(-0.5, 0.5), Vec2(0.5, -0.5)});

  win2d_1->add_ray({Vec2(0.0, 0.0), Vec2(-0.5, 0.5), Vec4(0.0, 1.0, 0.4, 1.0)});

  win2d_2->add_line({Vec2(-0.5, -0.5), Vec2(0.5, 0.5), Vec4(0.8, 0.0, 0.4, 1.0)});
  win2d_2->add_line({Vec2(-0.5, 0.5), Vec2(0.5, -0.5), Vec4(0.8, 0.0, 0.4, 1.0)});

  auto plotter = std::make_shared<Plot>();

  std::vector<double> vals;
  for (int k = 0; k < 10000; ++k) {
    // const double v = k % 20;
    const double v = Eigen::Vector2d::Random()[0];
    vals.push_back(v * v);
  }

  plotter->add_histogram({vals, {100}, {0.0, 1.0, 0.0, 0.8}});
  win2d_2->add_primitive(plotter);

  WindowManager::spin();
  std::cout << "Done" << std::endl;
}
}  // namespace viewer

int main() {
  viewer::run();
}
