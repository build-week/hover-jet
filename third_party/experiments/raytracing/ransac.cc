#include "raytracing/ransac.hh"
#include "geometry/perp.hh"

// TODO
#include "viewer/window_2d.hh"
#include "viewer/window_manager.hh"

#include <random>

namespace raytrace {
using Vec2 = Eigen::Vector2d;

// DirectedLineSegment: model
bool fit_model(const std::vector<Observation>&       observations,
               const std::vector<int>&               candidate_indices,
               Out<std::vector<DirectedLineSegment>> hypotheses) {
  if (candidate_indices.size() != 2) {
    // TODO: shit the bed
    return false;
  }

  const Observation& a = observations[candidate_indices[0]];
  const Observation& b = observations[candidate_indices[1]];

  const double a_b_dist = (a.hit - b.hit).norm();
  if (a_b_dist < 1e-3) {
    return false;
  }

  //
  // Check if observation rays point in opposite directions
  //

  const Vec2 a_dir = a.hit - a.origin;
  const Vec2 b_dir = b.hit - b.origin;

  if (a_dir.dot(b_dir) < 0.0) {
    return false;
  }

  const Vec2 center = (a.hit + b.hit) / 2.0;
  Vec2       normal = geometry::perp((a.hit - center).eval());

  //
  // Flip the normal if it is pointing away from the observations
  //

  const Vec2 direction = a.hit - a.origin;
  if (normal.dot(direction) > 0.0) {
    normal = -normal;
  }

  hypotheses->push_back(DirectedLineSegment(normal, center, a_b_dist));

  return true;
}

bool refine(DirectedLineSegment&            model,
            const std::vector<Observation>& observations,
            const std::vector<int>&         inlier_indices) {
  //
  // Refine model
  //

  Vec2 mean = Vec2::Zero();
  for (int k : inlier_indices) {
    mean += observations[k].hit;
  }
  mean /= static_cast<double>(inlier_indices.size());

  //
  // Refine width
  //

  double max_sq_dist = 0.0;
  for (int k : inlier_indices) {
    const double sq_dist = (observations[k].hit - mean).squaredNorm();
    if (sq_dist > max_sq_dist) {
      max_sq_dist = sq_dist;
    }
  }

  //
  // (Don't change normal)
  //

  model.center = mean;
  model.radius = std::sqrt(max_sq_dist);
  return true;
}

bool consistent_with_model(const DirectedLineSegment& model, const Observation& observation) {
  constexpr double MAX_DIST = 0.005f;

  // Could the poor guy be seen?
  const bool   was_viewable = (observation.hit - observation.origin).dot(model.normal) < 0.0;
  const double distance     = std::abs(model.to_plane_distance(observation.hit));

  if ((distance < MAX_DIST) && was_viewable) {
    return true;
  }
  return false;
}

// DirectedLineSegment: model
void find_inliers(const DirectedLineSegment&      line,
                  const std::vector<Observation>& observations,
                  const RansacConfiguration&      config,
                  Out<std::vector<int>>           inlier_indices) {
  inlier_indices->clear();

  //
  // Determine which indices were inliers for the model
  //

  for (size_t k = 0; k < observations.size(); ++k) {
    const bool was_consistent = consistent_with_model(line, observations[k]);

    if (was_consistent) {
      inlier_indices->push_back(k);
    }
  }
}

void discover_models(const std::vector<Observation>&       observations,
                     const RansacConfiguration&            config,
                     Out<std::vector<DirectedLineSegment>> models) {
  using Vec4 = Eigen::Vector4d;
  //
  // Random sampling with replacement
  //

  std::uniform_int_distribution<int> std_int(0, observations.size() - 1);
  std::mt19937                       gen(0);
  std::vector<int>                   inlier_to_test_indices(config.inliers_for_hypothesis);

  // int best_inlier_ct = 0;
  // TODO: priority heap ordered refine
  // TODO: min heap inlier count
  std::vector<DirectedLineSegment> to_test_models;
  std::vector<int>                 inlier_indices;

  auto win2d_1 = viewer::get_window2d("Ransac View");

  for (int test_num = 0; test_num < config.max_fit_ct; ++test_num) {
    // Attempt a series of model fits, followed by inlier counting and refinement

    //
    // Generate sample indices
    //

    for (int sample_num = 0; sample_num < config.inliers_for_hypothesis; ++sample_num) {
      inlier_to_test_indices[sample_num] = std_int(gen);
    }

    //
    // Attempt a fit
    //

    to_test_models.clear();
    const bool fit_succeeded = fit_model(observations, inlier_to_test_indices, out(to_test_models));

    if (!fit_succeeded) {
      continue;
    }

    //
    // Count inliers for the fit and refine the model
    //

    for (auto& model : to_test_models) {
      Eigen::Vector4d color;

      find_inliers(model, observations, config, out(inlier_indices));
      const int inliers_found = static_cast<int>(inlier_indices.size());
      if (inliers_found >= config.inliers_for_valid) {
        refine(model, observations, inlier_indices);
        models->push_back(model);
        color = Vec4(0.0, 1.0, 0.0, 1.0);

      } else {
        color = Vec4(1.0, 0.0, 0.0, 1.0);
      }

      //
      // Draw it
      //

      const Vec2 perp  = geometry::perp(model.normal);
      const Vec2 start = model.center + (perp * model.radius);
      const Vec2 end   = model.center - (perp * model.radius);
      win2d_1->add_line({start, end, color});

      viewer::WindowManager::draw(20);
    }
  }
  viewer::WindowManager::spin();
}
}
