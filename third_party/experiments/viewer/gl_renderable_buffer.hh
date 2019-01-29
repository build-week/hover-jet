#pragma once

#include "viewer/gl_size.hh"

namespace viewer {
//
// WARNING:
// I have not gotten this to work as a useful offscreen buffer.
//

class GlRenderableBuffer {
 public:
  GlRenderableBuffer(const GlSize& size);
  void bind();

 private:
  void initialize();
  bool initialized() const {
    return initialized_;
  }

  int frame_buffer_id_ = -1;
  int texture_target_ = -1;
  int depth_buffer_ = -1;
  bool initialized_ = false;

  GlSize size_;
};
}  // namespace viewer
