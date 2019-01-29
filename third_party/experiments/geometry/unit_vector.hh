#pragma once

#include "eigen.hh"

namespace geometry {

template <int N>
class UnitVector {
 public:
  static check(const VecNd<N>& v) {
    constexpr double F_EPS = 1e-6;
    assert(std::abs(v.squaredNorm() - 1.0) < F_EPS);
    return UnitVector(v);
  }

  static UnitVector unchecked(const VecNd<N>& v) {
    return UnitVector(v);
  }

 private:
  UnitVector(const VecNd<N>& v) {
    v_ = v;
  }

  VecNd<N> v_;
};

}  // namespace geometry