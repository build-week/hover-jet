#pragma once

namespace viewer {
class Primitive {
 public:
  virtual ~Primitive() = default;
  virtual void flush() {
  }
  virtual void flip() {
  }
  virtual void draw() const = 0;
};
}  // namespace viewer