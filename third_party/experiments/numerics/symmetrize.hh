#pragma once

#include "eigen.hh"

namespace numerics {
template <int rows>
MatNd<rows, rows> symmetrize(const MatNd<rows, rows>& mat) {
  return (mat + mat.transpose()) * 0.5;
}
}  // namespace numerics