#pragma once

//%deps(opengl)

#include "sophus.hh"
#include "viewer/primitives/primitive.hh"

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

// TODO
#include <stack>

namespace viewer {
namespace detail {

template <typename KeyType, typename ElementType>
class FrameTree {
 public:
  FrameTree(const KeyType& root_id) {
    root_id_ = root_id;
    adjacency_[root_id];
  }

  struct Edge {
    KeyType child_key;
    SE3 parent_from_child;
  };

  void add_child(const KeyType& key, const Edge& edge, const ElementType& child) {
    // TODO: Come up with a better way to support multiple references to the same mesh
    // Right now, we pass through the same submesh many times

    // assert(adjacency_.count(edge.child_key) == 0);
    adjacency_.at(key).push_back(edge);
    elements_[edge.child_key] = child;
    adjacency_[edge.child_key];
  }

  using VisitorFunc = std::function<void(const KeyType& parent_key,
                                         const ElementType& parent,
                                         const Edge& child_from_parent,
                                         const ElementType& child)>;
  void traverse(const VisitorFunc& visitor) const {
    std::stack<KeyType> to_visit;
    to_visit.push(root_id_);
    while (!to_visit.empty()) {
      const KeyType parent_key = to_visit.top();
      to_visit.pop();

      const auto& parent = elements_.at(parent_key);
      for (const auto& edge : adjacency_.at(parent_key)) {
        const ElementType& child = elements_.at(edge.child_key);
        visitor(parent_key, parent, edge, child);
        to_visit.push(edge.child_key);
      }
    }
  }

  const std::map<KeyType, ElementType>& elements() const {
    return elements_;
  }

  const std::map<KeyType, std::vector<Edge>>& adjacency() const {
    return adjacency_;
  }

 private:
  KeyType root_id_;

  std::map<KeyType, ElementType> elements_;
  std::map<KeyType, std::vector<Edge>> adjacency_;
};

}  // namespace detail

class SceneTree final : public Primitive {
  using KeyType = std::string;

 public:
  SceneTree() : tree_("root") {
  }

  void draw() const override;

  void set_world_from_root(const SE3& world_from_root) {
    const std::lock_guard<std::mutex> lk(draw_mutex_);
    world_from_root_ = world_from_root;
  }

  void add_primitive(const KeyType& parent,
                     const SE3& parent_from_child,
                     const KeyType& child_key,
                     const std::shared_ptr<Primitive> primitive) {
    const std::lock_guard<std::mutex> lk(draw_mutex_);
    tree_.add_child(parent, {child_key, parent_from_child}, primitive);
  }

  template <typename PrimitiveType, typename... Args>
  std::shared_ptr<PrimitiveType> add_primitive(const KeyType& parent,
                                               const SE3& parent_from_child,
                                               const KeyType& child_key,
                                               const Args&... args) {
    const std::lock_guard<std::mutex> lk(draw_mutex_);

    auto ptr = std::make_shared<PrimitiveType>(args...);
    tree_.add_child(parent, {child_key, parent_from_child}, ptr);
    return ptr;
  }

 private:
  void traverse(const KeyType& key) const;

  mutable std::mutex draw_mutex_;

  using FrameTreeType = detail::FrameTree<KeyType, std::shared_ptr<Primitive>>;
  FrameTreeType tree_;

  SE3 world_from_root_;
};
}  // namespace viewer
