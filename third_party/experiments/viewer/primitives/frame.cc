
#include "viewer/primitives/frame.hh"

#include <GL/glew.h>
#include "viewer/gl_aliases.hh"

namespace viewer {

Frame::Frame(const SE3& frame_from_parent) {
  update_transform(frame_from_parent);
}

void Frame::update_transform(const SE3& frame_from_parent) {
  frame_from_parent_ = frame_from_parent;
}

void Frame::add_primitive(std::shared_ptr<Primitive> primitive) {
  primitives_.emplace_back(primitive);
}

void Frame::draw() const {
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glTransform(frame_from_parent_.inverse());
  for (const auto& primitive : primitives_) {
    primitive->draw();
  }
  glPopMatrix();
}
}