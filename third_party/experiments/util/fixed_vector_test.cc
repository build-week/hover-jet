#include "util/fixed_vector.hh"

#include "testing/gtest.hh"

namespace jcc {

TEST(FixedVectorTest, push_back) {
  using TestType = FixedVector<int, 3>;
  TestType vec;

  ASSERT_EQ(vec.size(), 0u);
  ASSERT_EQ(vec.capacity(), 3u);
  // ASSERT_EQ(TestType::CAPACITY, 3u);
  ASSERT_TRUE(vec.empty());

  for (std::size_t k = 0u; k < vec.capacity(); ++k) {
    vec.push_back(k);
  }

  for (std::size_t k = 0u; k < vec.capacity(); ++k) {
    EXPECT_EQ(vec[k], k);
  }

  EXPECT_EQ(vec.size(), 3u);
}

TEST(FixedVectorTest, bracket_operator) {
  using TestType = FixedVector<int, 3>;
  TestType vec;

  ASSERT_EQ(vec.size(), 0u);
  ASSERT_EQ(vec.capacity(), 3u);
  // ASSERT_EQ(TestType::CAPACITY, 3u);
  ASSERT_TRUE(vec.empty());

  for (std::size_t k = 0u; k < vec.capacity(); ++k) {
    vec.push_back(k);
  }

  vec[2] = 10;

  EXPECT_EQ(vec[2], 10);
  EXPECT_EQ(vec[1], 1);
  EXPECT_EQ(vec.at(2), 10);
  EXPECT_THROW(vec.at(100), std::out_of_range);
  EXPECT_EQ(vec.size(), 3u);
}
}  // namespace jcc
