#pragma once

//%ignore

#include "eigen.hh"

#include <vector>

namespace geometry {
namespace spatial {

// Unimplemented
class Octree {
 public:
  using Vec3 = Eigen::Vector3d;
  void build(const std::vector<Vec3> &points);

 private:
  struct TreeElement {
    static constexpr int NUM_CHILDREN = 8;
    struct Node {
      std::array<int, NUM_CHILDREN> children;
    };
    struct Leaf {
      std::vector<int> members;
    };
    bool is_node = true;
    union {
      Node node;
      Leaf leaf;
    };
  };
  std::vector<TreeElement> nodes_;
  std::vector<Vec3> points_;
};
}  // namespace spatial
}  // namespace geometry
