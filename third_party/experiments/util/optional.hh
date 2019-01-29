#pragma once

#include <experimental/optional>

namespace jcc {
template <typename T>
using Optional = std::experimental::optional<T>;
}