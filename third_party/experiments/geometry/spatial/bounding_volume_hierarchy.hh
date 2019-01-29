#pragma once

#include "geometry/spatial/bounding_box.hh"
#include "geometry/spatial/volume.hh"

#include "out.hh"

#include <functional>
#include <vector>

namespace geometry {
namespace spatial {

constexpr int DIM = 3;
class BoundingVolumeHierarchy final : public Volume {
 public:
  struct TreeElement {
    struct Node {
      int left_child_index;
    };

    struct Leaf {
      int start;
      int end;
    };

    union {
      Node node;
      Leaf leaf;
    };
    BoundingBox<DIM> bounding_box;
    bool is_leaf = false;
  };

  struct AABB {
    BoundingBox<DIM> bbox;
    int volume_index;
  };

  using NodeBuildVisitorFunction = std::function<void(const BoundingBox<DIM> &box, int depth, bool leaf)>;
  using IntersectVisitorFunction = std::function<void(const TreeElement &tree_element, bool intersected)>;

  // Build a bvh
  // @param[in] volumes The volumes around which to build the hierarchy
  // @param[in] visitor A function that will be called on every new node
  void build(const std::vector<Volume *> &volumes,
             const NodeBuildVisitorFunction &visitor = [](const BoundingBox<DIM> &, int, bool) {});

  Intersection intersect(const Ray &ray, const IntersectVisitorFunction &visitor) const;
  Intersection intersect(const Ray &ray) const override {
    return intersect(ray, [](const TreeElement &tree_element, bool) {});
  }

  bool does_intersect(const Ray &ray) const override;
  BoundingBox<DIM> bounding_box() const override;

  // Expose the whole tree
  const std::vector<TreeElement> &tree() const {
    return tree_;
  }

  const std::vector<AABB> &aabb() const {
    return aabb_;
  }

 private:
  int add_node_and_children(std::vector<AABB> &bounding_boxes,
                            size_t node_index,
                            size_t begin,
                            size_t end,
                            int depth,
                            const NodeBuildVisitorFunction &visitor);

  double traverse_leaf(const TreeElement::Leaf &leaf, const Ray &ray, Out<int> closest) const;

  std::vector<TreeElement> tree_;
  std::vector<AABB> aabb_;
};
}  // namespace spatial
}  // namespace geometry
