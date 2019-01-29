#pragma once

#include "out.hh"
#include "raytracing/geometry.hh"

#include "eigen.hh"

#include <vector>

namespace raytrace {
struct RansacConfiguration {
  // How many inliers are needed for a model fit (exact)
  int inliers_for_hypothesis = 2;

  // How many attempts should we execute to find inliers
  int max_fit_ct = 25;

  // Need this many inliers to be happy
  int inliers_for_valid = 10;
};

//
// Non-general ransac procedures
//

struct Observation {
  using Vec2 = Eigen::Vector2d;

  Vec2 origin;
  Vec2 hit;
};

// DirectedLineSegment: model
bool fit_model(const std::vector<Observation>&       points,
               const std::vector<int>&               candidate_indices,
               Out<std::vector<DirectedLineSegment>> hypotheses);

bool consistent_with_model(const DirectedLineSegment& model, const Observation& observation);

// DirectedLineSegment: model
void find_inliers(const DirectedLineSegment&      model,
                  const std::vector<Observation>& observations,
                  const RansacConfiguration&      config,
                  Out<std::vector<int>>           inlier_indices);


// DirectedLineSegment: model
void discover_models(const std::vector<Observation>&       observations,
                     const RansacConfiguration&            config,
                     Out<std::vector<DirectedLineSegment>> models);
}