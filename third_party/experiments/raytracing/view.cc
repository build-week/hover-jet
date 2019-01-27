#include "raytracing/geometry.hh"

#include "viewer/window_2d.hh"
#include "viewer/window_manager.hh"

#include <limits>

namespace raytrace {

using Vec2 = Eigen::Vector2d;
using Vec4 = Eigen::Vector4d;

struct LidarConfig {
  int    beam_ct;
  double range_m;
  double separation_rad;
};

class Map {
  //
  //
  //

 public:
  void add_element(const LineSegment& segment) {
    line_segments_.push_back(segment);
  }

  const std::vector<LineSegment>& line_segments() const {
    return line_segments_;
  }

  // Fire a ray into the world, return point where ray met world
  //
  // Provided as an interface so that a future implementation can use a more effective search
  // (todo: bounding area hierarchy)
  //
  // Outputs
  // -------
  // @param[out] The intersection point (if it exists)
  // @returns true if an intersection was found, false if not
  //
  bool ray_intersect(const Ray& ray, const LidarConfig& config, Out<Vec2> result) const {
    const double max_dist_sq  = config.range_m * config.range_m;
    double       best_dist_sq = std::numeric_limits<double>::max();

    for (const auto& line_segment : line_segments_) {
      Vec2 intersection;
      if (ray_line_segment_intersection(ray, line_segment, out(intersection))) {
        const double dist_sq = (intersection - ray.origin).squaredNorm();

        if ((dist_sq < best_dist_sq) && (dist_sq < max_dist_sq)) {
          // todo: these copies are probably the most expensive thing in the loop
          *result      = intersection;
          best_dist_sq = dist_sq;
        }
      }
    }

    if (best_dist_sq != std::numeric_limits<double>::max()) {
      return true;
    } else {
      return false;
    }
  }

 private:
  //
  // Store map elements
  //
  std::vector<LineSegment> line_segments_;
};

void run() {
  viewer::WindowManager win_man;
  auto                     win2d_1 = std::make_shared<viewer::Window2D>();
  win_man.register_window(viewer::GlSize(640, 640), win2d_1, "Two Dimensional Debugging");

  Map map;
  for (int k = 0; k < 15; ++k) {
    constexpr double  RADIUS = 25.0;
    const LineSegment line_segment({Vec2::Random() * RADIUS, Vec2::Random() * RADIUS});
    map.add_element(line_segment);
    win2d_1->add_line({line_segment.start, line_segment.end});
  }

  LidarConfig config;
  config.beam_ct        = 16;
  config.range_m        = 35.0;
  config.separation_rad = 0.01;

  for (int k = 0; k < 50; ++k) {
    std::cout << "--" << std::endl;
    for (double p = 0.0; p < 2.0 * M_PI; p += 0.1) {
      const Vec2 origin(0.0, 0.0);
      const Vec2 direction(std::sin(p), std::cos(p));

      const Ray ray({origin, direction});

      Vec2 intersection;
      if (map.ray_intersect(ray, config, out(intersection))) {
        win2d_1->add_ray({ray.origin, ray.direction, Vec4(0.0, 1.0, 0.4, 1.0)});
        win2d_1->add_circle({intersection, 0.1});
        std::cout << intersection.transpose() << std::endl;
      }
      win_man.draw();
    }

    win2d_1->clear();
  }

  // todo: Make it possible to run this in the loop (I think we can do this?)
  win_man.spin();
}
}

int main() {
  raytrace::run();
}