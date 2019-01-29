#include <GL/glew.h>

#include "viewer/gl_aliases.hh"
#include "viewer/primitives/scene_tree.hh"

namespace viewer {

void SceneTree::traverse(const SceneTree::KeyType& key) const {
  if (tree_.elements().count(key)) {
    const auto& element = tree_.elements().at(key);
    element->draw();
  }

  for (const auto& child : tree_.adjacency().at(key)) {
    glTransform(child.parent_from_child);
    traverse(child.child_key);

    // TODO: Can we use the matrix stack for this?
    glTransform(child.parent_from_child.inverse());
  }
}

void SceneTree::draw() const {
  const std::lock_guard<std::mutex> lk(draw_mutex_);

  glPushMatrix();
  glTransform(world_from_root_);
  traverse("root");
  glPopMatrix();
}

}  // namespace viewer
