#include "viewer/gl_renderable_buffer.hh"

//%deps(opengl, glfw, glew)

#include <GL/glew.h>
#include <cassert>

#include <iostream>

namespace viewer {

GlRenderableBuffer::GlRenderableBuffer(const GlSize& size) {
  size_ = size;
}

void GlRenderableBuffer::initialize() {
  initialized_ = true;

  GLuint frame_buffer_id;
  glGenFramebuffers(1, &frame_buffer_id);
  frame_buffer_id_ = frame_buffer_id;
  glBindFramebuffer(GL_FRAMEBUFFER, frame_buffer_id);

  //
  // Texture
  //

  GLuint texture_target;
  glGenTextures(1, &texture_target);
  texture_target_ = texture_target;
  glBindTexture(GL_TEXTURE_2D, texture_target);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, size_.width, size_.height, 0, GL_RGBA,
               GL_UNSIGNED_BYTE, 0);

  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, texture_target, 0);

  glBindTexture(GL_TEXTURE_2D, 0);

  //
  // Depth buffer
  //

  GLuint depth_buffer;
  glGenRenderbuffers(1, &depth_buffer);
  depth_buffer_ = depth_buffer;

  glBindRenderbuffer(GL_RENDERBUFFER, depth_buffer);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, size_.width, size_.height);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER,
                            depth_buffer);
  glBindRenderbuffer(GL_RENDERBUFFER, 0);

  //
  // Complete the framebuffer
  //

  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER,
                            depth_buffer);

  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  assert(glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE);
}

void GlRenderableBuffer::bind() {
  if (!initialized()) {
    initialize();
  }

  glBindFramebuffer(GL_FRAMEBUFFER, frame_buffer_id_);
  glViewport(0, 0, size_.width, size_.height);
  // glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
  // glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  // glEnable(GL_DEPTH_TEST);
}
}  // namespace viewer
