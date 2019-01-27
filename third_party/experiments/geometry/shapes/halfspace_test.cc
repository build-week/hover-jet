#include "testing/gtest.hh"

#include "geometry/perp.hh"
#include "geometry/shapes/halfspace.hh"

namespace geometry {
namespace shapes {
using Vec3 = Eigen::Vector3d;

TEST(HalfSpace, is_it_correct_or_what) {
  //
  // Setup
  //

  const Vec3 normal = Vec3(1.0, 1.0, 1.0).normalized();
  constexpr double FRONT_OFFSET = 3.0;
  constexpr double BEHIND_OFFSET = -3.0;
  const Vec3 p_front = normal * FRONT_OFFSET;
  const Vec3 p_behind = normal * BEHIND_OFFSET;

  constexpr double d = 1.0;
  const Plane plane{normal, d};

  //
  // Action
  //

  const double dist_front = sd_halfspace(p_front, plane);
  const double dist_behind = sd_halfspace(p_behind, plane);

  //
  // Verification
  //

  EXPECT_FLOAT_EQ(dist_front, FRONT_OFFSET - d);
  EXPECT_FLOAT_EQ(dist_behind, BEHIND_OFFSET - d);
}

TEST(HalfSpace, offset) {
  //
  // Setup
  //

  const Vec3 normal = Vec3(1.0, 1.0, 1.0).normalized();

  constexpr double OFFSET_AMT = 1.0;
  const Vec3 offset = perp(normal) * OFFSET_AMT;

  constexpr double FRONT_OFFSET = 3.0;
  constexpr double BEHIND_OFFSET = -3.0;
  const Vec3 p_front = (normal * FRONT_OFFSET) + offset;
  const Vec3 p_behind = (normal * BEHIND_OFFSET) + offset;

  constexpr double d = 1.0;
  const Plane plane{normal, d};

  //
  // Action
  //

  const double dist_front = sd_halfspace(p_front, plane);
  const double dist_behind = sd_halfspace(p_behind, plane);

  //
  // Verification
  //

  EXPECT_FLOAT_EQ(dist_front, FRONT_OFFSET - d);
  EXPECT_FLOAT_EQ(dist_behind, BEHIND_OFFSET - d);
}

}  // namespace shapes
}  // namespace geometry