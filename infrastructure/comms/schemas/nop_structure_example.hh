#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "third_party/nop/structure.h"

// A simple structure with internal annotation.
struct SimpleType {
  std::uint32_t foo;
  std::string bar;
  std::vector<std::uint32_t> baz;
  NOP_STRUCTURE(SimpleType, foo, bar, baz);
};
