#include "raytracing/ransac.hh"
#include "testing/gtest.hh"

#include "eigen.hh"

// TODO
#include "viewer/window_2d.hh"

namespace raytrace {
using Vec2 = Eigen::Vector2d;

TEST(LineModel, fit_model) {
  std::vector<Observation> observations;
  const Vec2               origin = Vec2::Zero();
  observations.push_back({origin, Vec2(2.0, 1.0)});
  observations.push_back({origin, Vec2(2.0, -1.0)});

  const std::vector<int> indices = {0, 1};

  std::vector<DirectedLineSegment> hypotheses;
  const bool                       valid = fit_model(observations, indices, out(hypotheses));
  EXPECT_TRUE(valid);

  std::cout << hypotheses[0].normal.transpose() << std::endl;
  std::cout << hypotheses[0].center.transpose() << std::endl;
}

TEST(LineModel, test_detects) {
  //
  // Setup
  //

  const std::vector<Vec2> points = {
      Vec2(0.0, 1.23119),        Vec2(0.11819, 1.17796),   Vec2(0.228697, 1.1282),    Vec2(0.334284, 1.08065),
      Vec2(0.437281, 1.03427),   Vec2(0.539802, 0.9881),   Vec2(0.643919, 0.941214),  Vec2(0.751839, 0.892615),
      Vec2(0.866094, 0.841163),  Vec2(0.989799, 0.785456), Vec2(1.12703, 0.723658),   Vec2(1.28343, 0.653226),
      Vec2(1.46726, 0.570442),   Vec2(1.69133, 0.46954),   Vec2(1.97686, 0.340962),   Vec2(2.36204, 0.167504),
      Vec2(2.92366, -0.0854058), Vec2(3.8427, -0.499273),  Vec2(5.67315, -1.32357),   Vec2(6.50261, -2.22152),
      Vec2(6.42419, -2.94008),   Vec2(6.34036, -3.70814),  Vec2(6.24865, -4.54837),   Vec2(6.14576, -5.49114),
      Vec2(6.02696, -6.57955),   Vec2(5.88522, -7.87824),  Vec2(5.70929, -9.49022),   Vec2(0.823875, -19.7968),
      Vec2(-0.960201, -16.421),  Vec2(-2.2372, -14.0048),  Vec2(-3.21345, -12.1576),  Vec2(-3.99799, -10.6731),
      Vec2(-4.65415, -9.43154),  Vec2(-5.22152, -8.358),   Vec2(-5.7264, -7.4027),    Vec2(-6.18729, -6.53064),
      Vec2(-6.61791, -5.71583),  Vec2(-7.0291, -4.93781),  Vec2(-7.42995, -4.17934),  Vec2(-6.73207, -2.94511),
      Vec2(-5.71429, -1.84551),  Vec2(-5.0051, -1.07931),  Vec2(-4.47343, -0.504892), Vec2(-4.05258, -0.0502099),
      Vec2(-3.7049, 0.325423),   Vec2(-3.40737, 0.646868), Vec2(-3.145, 0.930332),    Vec2(-2.90743, 1.187),
      Vec2(-2.68711, 1.42504),   Vec2(-2.4782, 1.65074),   Vec2(-2.27592, 1.86928),   Vec2(-2.07603, 2.08524),
      Vec2(-1.58196, 1.94358),   Vec2(-1.15556, 1.75156),  Vec2(-0.845797, 1.61207),  Vec2(-0.60638, 1.50425),
      Vec2(-0.412315, 1.41686),  Vec2(-0.248853, 1.34325), Vec2(-0.106658, 1.27922)};

  const Vec2 origin = Vec2::Zero();

  auto win2d_1 = viewer::get_window2d("Ransac View");


  std::vector<Observation> observations(points.size());
  for (size_t k = 0; k < points.size(); ++k) {
    observations[k] = {origin, points[k]};
    win2d_1->add_circle({points[k], 0.1});
  }

  RansacConfiguration config;
  config.inliers_for_valid = 6;
  config.max_fit_ct        = 200;

  std::vector<DirectedLineSegment> models;
  discover_models(observations, config, out(models));

  std::cout << models.size() << std::endl;
}
}
