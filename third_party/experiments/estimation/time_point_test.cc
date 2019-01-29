#include "estimation/time_point.hh"

#include "util/heap.hh"

#include "testing/gtest.hh"

namespace estimation {

TEST(TimePointTest, test_it_all_constexpr) {
  constexpr double TIME = 3.54e-5;

  constexpr auto dur = to_duration(TIME);
  constexpr double secs = to_seconds(dur);

  constexpr double EPS = 0.015;
  EXPECT_LT(std::abs(secs - TIME) / TIME, EPS);
}

TEST(TimePointTest, test_it_all) {
  constexpr double TIME = 3.54e-5;

  const auto dur = to_duration(TIME);
  const double secs = to_seconds(dur);

  constexpr double EPS = 0.015;
  EXPECT_LT(std::abs(secs - TIME) / TIME, EPS);
}

TEST(TimePointTest, sortable) {
  Heap<TimePoint> heap;

  const TimePoint start = TimePoint{};
  heap.push(start + to_duration(0.1));
  heap.push(start + to_duration(0.2));
  heap.push(start + to_duration(0.5));
  heap.push(start + to_duration(0.9));

  EXPECT_DOUBLE_EQ(to_seconds(heap.pop() - start), 0.9);
  EXPECT_DOUBLE_EQ(to_seconds(heap.pop() - start), 0.5);
  EXPECT_DOUBLE_EQ(to_seconds(heap.pop() - start), 0.2);
  EXPECT_DOUBLE_EQ(to_seconds(heap.pop() - start), 0.1);
}

}  // namespace estimation