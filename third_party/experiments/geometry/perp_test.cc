#include "geometry/perp.hh"
#include "testing/gtest.hh"

using Vec3 = Eigen::Vector3d;
namespace geometry {

TEST(Perp, is_perp) {
  EXPECT_TRUE(is_perp(Vec3(Vec3::UnitX().eval()), Vec3(Vec3::UnitY().eval())));
  EXPECT_TRUE(is_perp(Vec3(Vec3::UnitX().eval()), Vec3(Vec3::UnitZ().eval())));
  EXPECT_TRUE(is_perp(Vec3(1.0, 1.0, 0.0), Vec3(0.0, 0.0, 1.0)));

  EXPECT_FALSE(is_perp(Vec3::UnitX().eval(), Vec3(Vec3::UnitX().eval())));
  EXPECT_FALSE(is_perp(Vec3(Vec3::UnitX().eval()), Vec3(1.0, 1.0, 1.0)));
}

TEST(Perp, perp) {
  const Vec3 a = Vec3::UnitX();
  EXPECT_TRUE(is_perp(perp(a), a));

  constexpr int NUM_TESTS = 200;

  for (int k = 0; k < NUM_TESTS; ++k) {
    const Vec3 x = Vec3::Random() * k;
    const Vec3 perp_x = perp(x);
    EXPECT_TRUE(is_perp(perp_x, x));
  }
}
}  // namespace geometry
