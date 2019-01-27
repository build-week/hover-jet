#include "geometry/ray_ray_approx_intersect.hh"
#include "testing/gtest.hh"

namespace geometry {

using Vec3 = Eigen::Vector3d;

TEST(ApproximateIntersection, identical) {
  const Ray a{Vec3(0.0, 0.0, 0.0), Vec3(1.0, 0.0, 0.0)};
  const Ray b{Vec3(0.0, 0.0, 0.0), Vec3(1.0, 0.0, 0.0)};

  const auto result = ray_ray_approx_intersect(a, b);

  EXPECT_FALSE(result.valid);
}

TEST(ApproximateIntersection, shared_origin) {
  const Ray a{Vec3(0.0, 0.0, 0.0), Vec3(1.0, 0.0, 0.0)};
  const Ray b{Vec3(0.0, 0.0, 0.0), Vec3(0.0, 1.0, 0.0)};

  const auto result = ray_ray_approx_intersect(a, b);

  constexpr double EPS = 1e-6;
  EXPECT_NEAR(result.along_a, 0, EPS);
  EXPECT_NEAR(result.along_b, 0, EPS);
  EXPECT_TRUE(result.valid);
}

TEST(ApproximateIntersection, sensible_rays) {
  const Ray a{Vec3(0.0, 0.0, 0.0), Vec3(1.0, 0.0, 0.0)};
  const Ray b{Vec3(1.0, 1.0, 1.0), Vec3(-1.0, -1.0, 0.0)};

  const auto result = ray_ray_approx_intersect(a, b);

  EXPECT_TRUE(result.valid);

  const Vec3 result_a = a(result.along_a);
  const Vec3 result_b = b(result.along_b);

  constexpr double EPS = 1e-6;
  EXPECT_NEAR(result_a.x(), 0.0, EPS);
  EXPECT_NEAR(result_a.y(), 0.0, EPS);
  EXPECT_NEAR(result_a.z(), 0.0, EPS);

  EXPECT_NEAR(result_b.x(), 0.0, EPS);
  EXPECT_NEAR(result_b.y(), 0.0, EPS);
  EXPECT_NEAR(result_b.z(), 1.0, EPS);
}
}