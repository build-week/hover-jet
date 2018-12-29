#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "third_party/nop/structure.h"

// A simple structure with internal annotation.
struct CameraImage {
  std::vector<char> data;
  uint16_t height;
  uint16_t width;
  uint8_t channels;
  uint8_t depth_bits;

  NOP_STRUCTURE(CameraImage, data, height, width, channels, depth_bits);
};
