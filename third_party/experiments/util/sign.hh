#pragma once

#include <cmath>

namespace jcc {

//
// sign returns 1 or -1 for all inputs.
// The sign of 0 is 1. The sign of -0 is -1.
//

inline int sign(int x) {
  return (x >= 0) - (x < 0);
}

inline int sign(double val) {
  return std::copysign(1.0, val);
}

inline int sign(float val) {
  return std::copysign(1.0f, val);
}

}  // namespace jcc
