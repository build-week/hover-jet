#pragma once

#include "out.hh"

#include <algorithm>
#include <functional>
#include <vector>

// Max heap
template <typename T>
class Heap {
 public:
  using CompareFunction = std::function<bool(const T &a, const T &b)>;

  Heap(const CompareFunction &cmp = [](const T &a,
                                       const T &b) { return std::less<T>()(a, b); })
      : cmp_(cmp) {
  }

  Heap(const std::vector<T> &vector,
       const CompareFunction &cmp = [](const T &a,
                                       const T &b) { return std::less<T>()(a, b); })
      : cmp_(cmp), vector_(vector) {
    std::make_heap(vector_, cmp_);
  }

  Heap(const Heap<T> &o_heap) : cmp_(o_heap.cmp_), vector_(o_heap.vector_) {
  }

  void drop() {
    std::pop_heap(vector_.begin(), vector_.end(), cmp_);
    vector_.pop_back();
  }

  T pop() {
    std::pop_heap(vector_.begin(), vector_.end(), cmp_);
    const T result = vector_.back();
    vector_.pop_back();
    return result;
  }

  const T &top() {
    return vector_.front();
  }

  void push(const T &item) {
    vector_.push_back(item);
    std::push_heap(vector_.begin(), vector_.end(), cmp_);
  }

  void clear() {
    vector_.clear();
  }

  size_t size() const {
    return vector_.size();
  }

  bool empty() const {
    return vector_.empty();
  }

  std::vector<T> to_sorted_vector() const {
    Heap heap2 = *this;
    std::vector<T> result;
    while (!heap2.empty()) {
      result.push_back(heap2.top());
      heap2.drop();
    }
    return result;
  }

 private:
  CompareFunction cmp_;
  std::vector<T> vector_;
};
