#pragma once
//%deps(opengl)

#include "viewer/primitives/primitive.hh"

namespace viewer {

class Box final : public Primitive {
 public:
  void draw() const override;
};
}
