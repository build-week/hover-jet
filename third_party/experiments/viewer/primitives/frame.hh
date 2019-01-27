#pragma once

//%deps(opengl)

#include "viewer/primitives/primitive.hh"
#include "sophus.hh"

#include <memory>
#include <vector>

namespace viewer {

class Frame : public Primitive {
 public:
  Frame(const SE3& frame_from_parent);

  // TODO: Support
  // Frame(const Sim3& frame_from_parent);

  virtual void update_transform(const SE3& frame_from_parent);

  virtual void add_primitive(std::shared_ptr<Primitive> primitive);

  virtual void draw() const override;

 protected:
  std::vector<std::shared_ptr<Primitive>> primitives_;
  SE3                                     frame_from_parent_;

 private:
};
}
