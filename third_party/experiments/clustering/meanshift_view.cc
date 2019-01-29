#include "clustering/spatial_hash.hh"

#include "viewer/window_2d.hh"

#include "out.hh"

#include "eigen.hh"
#include <map>

namespace clustering {
using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;

struct MeanShiftState {
  std::vector<Vec2> initial_points;
  std::vector<Vec2> current_points;
};

Vec2 local_mean(const std::vector<Vec2> &points,
                const int index,
                const double window_size) {
  Vec2 mean = Vec2::Zero();
  int added = 0;
  for (const auto &pt : points) {
    const double dist = (pt - points[index]).norm();
    if (dist < window_size) {
      mean += pt;
      ++added;
    }
  }
  mean /= static_cast<double>(added);
  return mean;
}

void shift_means(Vout<std::vector<Vec2>> points, const double radius) {
  constexpr double DAMPING = 0.1;

  for (size_t k = 0; k < points->size(); ++k) {
    const Vec2 lmean = local_mean(*points, k, radius);
    const Vec2 direction = lmean - points[k];
    points[k] += DAMPING * direction;
  }
}

void run() {
  auto viewer = viewer::get_window2d("Meanshift View");

  constexpr int NUM_PTS = 500;
  std::vector<Vec2> points(NUM_PTS);
  for (int k = 0; k < 250; ++k) {
    points[k] = (Vec2::Random() * 2.0) + Vec2(-1, -1.5);
  }

  for (int k = 250; k < NUM_PTS; ++k) {
    points[k] = (Vec2::Random() * 2.0) + Vec2(1, 1.5);
  }

  std::vector<Vec2> deformed_pts(points);

  constexpr double MEANSHIFT_RADIUS = 1.0;
  for (int k = 0; k < 100; ++k) {
    viewer->clear();
    viewer->add_points({points, Vec4(0.0, 1.0, 0.0, 0.8)});
    shift_means(vout(deformed_pts), MEANSHIFT_RADIUS);
    viewer->add_points({deformed_pts, Vec4(1.0, 0.0, 0.0, 0.8)});
    viewer->spin_until_step();
  }

  //
  // Compute the cluster identities using an lsh
  //

  const std::vector<HashInt> identities = spatial_hash(deformed_pts, 100.0);
  std::map<HashInt, std::vector<Vec2>> identified_points;
  for (size_t k = 0; k < identities.size(); ++k) {
    const HashInt id = identities[k];
    identified_points[id].push_back(points[k]);
  }

  //
  // Draw the identified points
  //

  viewer->clear();
  for (const auto &pt_group : identified_points) {
    const Vec3 color = (Vec3::Random() + Vec3::Ones()) * 0.5;
    const Vec4 color_and_alpha = (Vec4() << color, 1.0).finished();
    viewer->add_points({pt_group.second, color_and_alpha});
  }
  viewer->spin_until_step();
}
}  // namespace clustering

int main() {
  clustering::run();
}
