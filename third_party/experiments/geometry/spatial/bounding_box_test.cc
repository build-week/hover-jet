
#include "testing/gtest.hh"

#include "geometry/spatial/bounding_box.hh"
#include "eigen.hh"

using Vec3 = Eigen::Vector3d;

namespace geometry {
namespace spatial {

TEST(TestBoundingBox, intersection_test) {
  BoundingBox<3> bbox;
  bbox.expand(Vec3(0.0, 0.0, 0.0));
  bbox.expand(Vec3(1.0, 1.0, 1.0));

  {  // Translated beneath on y, pointing at box
    geometry::Ray ray;
    ray.origin        = Vec3(0.0, -0.5, 0.5);
    ray.direction     = Vec3(0.0, 1.0, 0.0);
    const auto result = bbox.intersect(ray);

    EXPECT_TRUE(result.intersected);

    constexpr double EPS = 1e-6;
    ASSERT_FALSE(bbox.contains(ray.origin));
    EXPECT_NEAR(result.distance, 0.5, EPS);
  }

  {  // Translated above on y, pointing at box
    geometry::Ray ray;
    ray.origin        = Vec3(0.0, 1.5, 0.5);
    ray.direction     = Vec3(0.0, -1.0, 0.0);
    const auto result = bbox.intersect(ray);

    EXPECT_TRUE(result.intersected);

    constexpr double EPS = 1e-6;
    ASSERT_FALSE(bbox.contains(ray.origin));
    EXPECT_NEAR(result.distance, 0.5, EPS);
  }

  {  // translated above on z, pointing at box
    geometry::Ray ray;
    ray.origin        = Vec3(0.5, 0.0, 1.5);
    ray.direction     = Vec3(0.0, 0.0, -1.0);
    const auto result = bbox.intersect(ray);

    EXPECT_TRUE(result.intersected);

    constexpr double EPS = 1e-6;
    ASSERT_FALSE(bbox.contains(ray.origin));
    EXPECT_NEAR(result.distance, 0.5, EPS);
  }

  {  // translated above on z, pointing at box
    geometry::Ray ray;
    ray.origin        = Vec3(0.5, 0.0, -0.5);
    ray.direction     = Vec3(0.0, 0.0, 1.0);
    const auto result = bbox.intersect(ray);

    EXPECT_TRUE(result.intersected);

    constexpr double EPS = 1e-6;
    ASSERT_FALSE(bbox.contains(ray.origin));
    EXPECT_NEAR(result.distance, 0.5, EPS);
  }

  {  // translated above on x, pointing at box
    geometry::Ray ray;
    ray.origin        = Vec3(-0.5, 0.0, 0.5);
    ray.direction     = Vec3(1.0, 0.0, 0.0);
    const auto result = bbox.intersect(ray);

    EXPECT_TRUE(result.intersected);

    constexpr double EPS = 1e-6;
    ASSERT_FALSE(bbox.contains(ray.origin));
    EXPECT_NEAR(result.distance, 0.5, EPS);
  }

  {  // translated above on x, pointing at box
    geometry::Ray ray;
    ray.origin        = Vec3(1.5, 0.0, 0.5);
    ray.direction     = Vec3(-1.0, 0.0, 0.0);
    const auto result = bbox.intersect(ray);

    EXPECT_TRUE(result.intersected);

    constexpr double EPS = 1e-6;
    ASSERT_FALSE(bbox.contains(ray.origin));
    EXPECT_NEAR(result.distance, 0.5, EPS);
  }

  {
    geometry::Ray ray;
    ray.origin        = Vec3(0.0, -0.5, 0.5);
    ray.direction     = Vec3(0.0, -1.0, 0.0);
    const auto result = bbox.intersect(ray);

    ASSERT_FALSE(bbox.contains(ray.origin));
    EXPECT_FALSE(result.intersected);
  }

  {
    geometry::Ray ray;
    ray.origin        = Vec3(0.0, -1.0, 0.5);
    ray.direction     = Vec3(0.0, -1.0, -1.0).normalized();
    const auto result = bbox.intersect(ray);

    ASSERT_FALSE(bbox.contains(ray.origin));
    EXPECT_FALSE(result.intersected);
  }
}

TEST(TestBoundingBox, contains_ray) {
  BoundingBox<3> bbox;
  bbox.expand(Vec3(0.0, 0.0, 0.0));
  bbox.expand(Vec3(1.0, 1.0, 1.0));

  {  // Translated beneath on y, pointing at box
    geometry::Ray ray;
    ray.origin        = Vec3(0.5, 0.5, 0.5);
    ray.direction     = Vec3(0.0, 1.0, 0.0);
    const auto result = bbox.intersect(ray);

    EXPECT_TRUE(result.intersected);

    constexpr double EPS = 1e-6;
    ASSERT_TRUE(bbox.contains(ray.origin));
    EXPECT_NEAR(result.distance, 0.5, EPS);
  }
}

TEST(TestBoundingBox, does_not_intersect) {
  BoundingBox<3> bbox;
  bbox.expand(Vec3(0.0, 0.0, 0.0));
  bbox.expand(Vec3(1.0, 1.0, 1.0));

  {
    geometry::Ray ray;
    ray.origin        = Vec3(1.5, 1.5, 1.5);
    ray.direction     = Vec3(0.0, 1.0, 0.0);
    const auto result = bbox.intersect(ray);

    ASSERT_FALSE(bbox.contains(ray.origin));
    EXPECT_FALSE(result.intersected);
  }

  {
    geometry::Ray ray;
    ray.origin        = Vec3(0.0, 0.0, 1.5);
    ray.direction     = Vec3(0.0, 1.0, 0.0);
    const auto result = bbox.intersect(ray);

    ASSERT_FALSE(bbox.contains(ray.origin));
    EXPECT_FALSE(result.intersected);
  }
}

// Test that we fixed a bug found in testing the bounding volume hierarchy
// (Turns out, the slab method as described fails when parallel to the box)
TEST(TestBoundingBox, apparent_bug) {
  BoundingBox<3> bbox;
  bbox.expand(Vec3(0.534403, -0.364025, 8.25298));
  bbox.expand(Vec3(-2.01473, -3.04393, 2.45791));

  geometry::Ray ray;
  ray.origin                      = Vec3(0.0, 0.0, 0.75);
  ray.direction                   = Vec3(-1.0, -1.0, 0.0).normalized();
  const Intersection intersection = bbox.intersect(ray);

  EXPECT_FALSE(intersection.intersected);
}
}
}
