#pragma once

#include <vector>
#include <string>

namespace slam {

struct DepthAndImageLocation {
  std::string image;
  std::string depth;
};

std::vector<std::string> get_images();
std::vector<std::string> get_depths();
std::vector<DepthAndImageLocation> get_both();
}
