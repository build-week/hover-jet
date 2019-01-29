#include "testing/gtest.hh"

#include <limits>
#include "util/sign.hh"

namespace jcc {

TEST(Sign, sgn_double) {
  EXPECT_EQ(sign(5.0), 1);
  EXPECT_EQ(sign(-5.0), -1);

  EXPECT_EQ(sign(std::numeric_limits<double>::lowest()), -1);
  EXPECT_EQ(sign(std::numeric_limits<double>::min()), 1);
  EXPECT_EQ(sign(std::numeric_limits<double>::max()), 1);
  EXPECT_EQ(sign(std::numeric_limits<double>::infinity()), 1);
  EXPECT_EQ(sign(-0.0), -1);
  EXPECT_EQ(sign(0.0), 1);
}

TEST(Sign, sgn_float) {
  EXPECT_EQ(sign(std::numeric_limits<float>::lowest()), -1);
  EXPECT_EQ(sign(std::numeric_limits<float>::min()), 1);
  EXPECT_EQ(sign(std::numeric_limits<float>::max()), 1);
  EXPECT_EQ(sign(std::numeric_limits<float>::infinity()), 1);
  EXPECT_EQ(sign(-0.0f), -1);
  EXPECT_EQ(sign(0.0f), 1);
}

TEST(Sign, sgn_int) {
  EXPECT_EQ(sign(5), 1);
  EXPECT_EQ(sign(-5), -1);
  EXPECT_EQ(sign(0), 1);  // Eek
}

}  // namespace jcc
