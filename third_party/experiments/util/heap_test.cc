#include "testing/gtest.hh"

#include "util/heap.hh"

namespace jcc {
TEST(Heap, pushpop) {
  Heap<int> heap;

  heap.push(1);
  heap.push(0);
  heap.push(2);

  EXPECT_EQ(heap.top(), 2);
  EXPECT_EQ(heap.pop(), 2);

  EXPECT_EQ(heap.top(), 1);
  EXPECT_EQ(heap.pop(), 1);

  EXPECT_EQ(heap.top(), 0);
  EXPECT_EQ(heap.pop(), 0);
}

TEST(Heap, pushdrop) {
  Heap<int> heap;

  heap.push(1);
  heap.push(0);
  heap.push(2);

  heap.drop();
  heap.drop();
  EXPECT_EQ(heap.top(), 0);
}

TEST(Heap, alt_cmp) {
  const auto cmp = [](const int &a, const int &b) { return a > b; };
  Heap<int> heap(cmp);

  heap.push(1);
  heap.push(0);
  heap.push(2);

  EXPECT_EQ(heap.top(), 0);
  EXPECT_EQ(heap.pop(), 0);

  EXPECT_EQ(heap.top(), 1);
  EXPECT_EQ(heap.pop(), 1);

  EXPECT_EQ(heap.top(), 2);
  EXPECT_EQ(heap.pop(), 2);
}

TEST(Heap, pushpoppushpop) {
  Heap<int> heap;

  heap.push(1);
  heap.push(0);
  heap.push(2);

  EXPECT_EQ(heap.pop(), 2);
  EXPECT_EQ(heap.top(), 1);

  heap.push(8);
  EXPECT_EQ(heap.top(), 8);
  heap.pop();
  EXPECT_EQ(heap.top(), 1);
  heap.push(20);
  heap.push(75);
  EXPECT_EQ(heap.top(), 75);
  heap.pop();
  heap.pop();
  heap.pop();
  EXPECT_EQ(heap.top(), 0);
}

TEST(Heap, pop) {
  Heap<int> heap;

  heap.push(1);
  heap.push(0);
  heap.push(2);

  EXPECT_EQ(heap.pop(), 2);
  EXPECT_EQ(heap.pop(), 1);
  EXPECT_EQ(heap.pop(), 0);
}
}  // namespace jcc
