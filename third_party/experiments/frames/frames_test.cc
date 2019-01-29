#include "testing/gtest.hh"

#include "frames/frames.hh"

namespace frames {

class JakesFiniteField {
 public:
  JakesFiniteField(int v) {
    v_ = v % 2;
  }

  JakesFiniteField operator*(const JakesFiniteField& f) const {
    return v_ * f.v();
  }

  int v() const {
    return v_;
  }

  JakesFiniteField inverse() const {
    return v_ + 1 % 2;
  }

 private:
  int v_;
};

TEST(Frames, frames) {
  const Transform<FrameId::CAMERA, FrameId::BODY, JakesFiniteField> camera_from_body(1);

  const auto a = camera_from_body.inverse() * camera_from_body;
  (void)a;
  const auto b = camera_from_body * camera_from_body.inverse();
  (void)b;

  const Transform<FrameId::BODY, FrameId::WORLD, JakesFiniteField> body_from_world(0);

  const auto camera_from_world = camera_from_body * body_from_world;

  const auto c = camera_from_world.inverse() * camera_from_body;
  (void)c;
  const auto d = camera_from_world * body_from_world.inverse();
  (void)d;
}
}