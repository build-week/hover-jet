#pragma once

namespace jcc {
template <typename T>
const T &clamp(const T &value, const T &min, const T &max) {
  return value < min ? min : (value > max ? max : value);
}
}
