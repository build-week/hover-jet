#include "testing/gtest.hh"

#include "util/clamp.hh"

namespace jcc {
TEST(Clamp, works) {

  constexpr double a = 1.0;
  constexpr double b = -1.0;
  constexpr double c = 2.0;
  constexpr double d = 3.0;

  constexpr double min = 0.5;
  constexpr double max = 2.0;

  EXPECT_EQ(clamp(a, min, max), a);
  EXPECT_EQ(clamp(b, min, max), min);
  EXPECT_EQ(clamp(c, min, max), c);
  EXPECT_EQ(clamp(d, min, max), max);
}
}
