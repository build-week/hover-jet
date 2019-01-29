#pragma once

#include <array>

namespace jcc {
template <class T, std::size_t N>
class FixedVector {
 private:
  std::array<T, N> backing_;
  std::size_t size_ = 0u;

 public:
  static constexpr std::size_t CAPACITY = N;
  using MemberType = T;

  FixedVector() = default;

  void push_back(const T& value) {
    // assert(size_ + 1 < CAPACITY, "FixedVector is full!");
    if (size_ + 1 > CAPACITY) {
      throw std::out_of_range("Out of bounds push_back on FixedVector");
    }
    backing_[size_] = value;
    size_++;
  }

  const T& at(std::size_t pos) const {
    if (pos >= size_) {
      throw std::out_of_range("Out of bounds access on stackvector");
    }
    return backing_[pos];
  }

  T& at(std::size_t pos) {
    if (pos >= size_) {
      throw std::out_of_range("Out of bounds access on stackvector");
    }
    return backing_[pos];
  }

  // No bounds checking
  const T& operator[](std::size_t pos) const {
    return backing_[pos];
  }

  // No bounds checking
  // Note: You can do out-of-order assignment and thereby make size() invalid
  // Don't do that!
  T& operator[](std::size_t pos) {
    return backing_[pos];
  }

  // Lazy! Doesn't destroy the last object
  void pop_back() {
    size_--;
  }

  std::size_t size() const {
    return size_;
  }

  std::size_t capacity() const {
    return CAPACITY;
  }

  bool empty() const {
    return size_ == 0u;
  }
};
}  // namespace jcc