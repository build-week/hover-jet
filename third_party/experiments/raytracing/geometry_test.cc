#include "raytracing/geometry.hh"
#include "numerics/numdiff.hh"

#include "testing/gtest.hh"

namespace rt = raytrace;
namespace nm = numerics;

using Vec2 = Eigen::Vector2d;

// Ray pointing at an obvious intersection point
//
TEST(Primitives, ray_line_intersection_nominal) {
  constexpr double EPS = 1e-6;

  //
  // Setup
  //

  const rt::Ray ray(Vec2(0.5, 0.0), Vec2(1.0, 0.0));

  // Equivalent lines
  const rt::Line line(Vec2(1.0, 0.0), Vec2(0.0, 1.0));
  const rt::Line line2(Vec2(1.0, 0.0), Vec2(0.0, -1.0));

  //
  // Action
  //

  Vec2 intersection;
  bool intersected = rt::ray_line_intersection(ray, line, out(intersection));

  Vec2 intersection2;
  bool intersected2 = rt::ray_line_intersection(ray, line2, out(intersection2));

  //
  // Verification
  //

  const Vec2 expected_intersection(1.0, 0.0);
  EXPECT_TRUE(intersected);
  EXPECT_TRUE(intersected2);

  EXPECT_LT((intersection - expected_intersection).norm(), EPS);
  EXPECT_LT((intersection - intersection2).norm(), EPS);
}

// The ray is pointing the wrong direction
//
TEST(Primitives, ray_line_intersection_backwards) {
  constexpr double EPS = 1e-6;

  //
  // Setup
  //

  const rt::Ray  ray(Vec2(0.0, 0.0), Vec2(-1.0, 0.0));
  const rt::Line line(Vec2(1.0, 0.0), Vec2(0.0, 1.0));

  //
  // Action
  //

  Vec2 intersection;
  bool intersected = rt::ray_line_intersection(ray, line, out(intersection));

  //
  // Verification
  //

  const Vec2 expected_intersection(1.0, 0.0);

  // Intersection should have failed, but still expect to return the
  // place where the reversed ray would have intersected the line
  EXPECT_FALSE(intersected);
  EXPECT_LT((intersection - expected_intersection).norm(), EPS);
}

// The ray is parallel to the line
//
TEST(Primitives, ray_line_intersection_parallel) {
  //
  // Setup
  //

  const rt::Ray  ray(Vec2(0.0, 0.0), Vec2(0.0, 1.0));
  const rt::Line line(Vec2(1.0, 0.0), Vec2(0.0, 1.0));

  //
  // Action
  //

  Vec2 intersection;
  bool intersected = rt::ray_line_intersection(ray, line, out(intersection));

  //
  // Verification
  //

  // No intersection if the lines are parallel (Up to precision)
  EXPECT_FALSE(intersected);
}

// Ray pointed at line segment
//
TEST(Primitives, ray_line_segment_intersection) {
  constexpr double EPS = 1e-6;

  //
  // Setup
  //

  const rt::Ray ray(Vec2(0.0, 1.0), Vec2(0.0, -1.0));

  // Same line segment with switched ordering
  const rt::LineSegment segment(Vec2(1.0, 0.0), Vec2(-1.0, 0.0));
  const rt::LineSegment segment2(Vec2(-1.0, 0.0), Vec2(1.0, 0.0));

  //
  // Action
  //

  Vec2 intersection;
  bool intersected = rt::ray_line_segment_intersection(ray, segment, out(intersection));

  Vec2 intersection2;
  bool intersected2 = rt::ray_line_segment_intersection(ray, segment2, out(intersection2));

  //
  // Verification
  //

  // Both rays should have hit
  EXPECT_TRUE(intersected);
  EXPECT_TRUE(intersected2);

  // They should intersect ta the same point
  EXPECT_LT((intersection - intersection2).norm(), EPS);

  // And that point should be the origin
  const Vec2 expected_intersection(0.0, 0.0);
  EXPECT_LT((intersection - expected_intersection).norm(), EPS);
}

// Verify that distance from point to plane and its derivative is correct
//
//
TEST(Primitives, plane_distance) {
  constexpr double EPS = 1e-6;

  //
  // Setup
  //

  // Should be normalized in the constructor
  const Vec2 normal(1.9, 0.0);
  const Vec2 center(0.0, 0.0);
  const auto segment = rt::DirectedLineSegment(normal, center, 1.0);

  //
  // Action
  //

  const Vec2 test(0.976, 0.25);
  Vec2       analytical_grad = segment.dto_plane_distance(test);

  double dist        = segment.to_plane_distance(test);
  double dist_behind = segment.to_plane_distance(-test);

  //
  // Verification
  //

  EXPECT_NEAR(dist, test(0), EPS);
  EXPECT_NEAR(dist_behind, -dist, EPS);

  const auto fcn             = std::bind(&rt::DirectedLineSegment::to_plane_distance, &segment, std::placeholders::_1);
  const Vec2 numerical_dcost = nm::numerical_gradient(test, fcn);
  EXPECT_LT((analytical_grad - numerical_dcost).norm(), EPS);
}

// Verify that distance projected along plane and its derivative is correct
//
//
TEST(Primitives, projected_plane_distance) {
  constexpr double EPS = 1e-6;

  //
  // Setup
  //

  // Should be normalized in the constructor
  const Vec2 normal(1.9, 0.0);
  const Vec2 center(0.0, 0.0);
  const auto segment = rt::DirectedLineSegment(normal, center, 1.0);

  //
  // Action
  //

  const Vec2 test(0.976, 0.25);
  Vec2       analytical_grad = segment.dalong_plane_distance(test);

  double dist        = segment.along_plane_distance(test);
  double dist_behind = segment.along_plane_distance(-test);

  //
  // Verification
  //

  EXPECT_NEAR(dist, test(1), EPS);
  EXPECT_NEAR(dist_behind, -dist, EPS);

  const auto fcn = std::bind(&rt::DirectedLineSegment::along_plane_distance, &segment, std::placeholders::_1);
  const Vec2 numerical_dcost = nm::numerical_gradient(test, fcn);
  EXPECT_LT((analytical_grad - numerical_dcost).norm(), EPS);
}
