#include "viewer/simple_window.hh"

//%deps(OPENGL, GLFW)

#include <GLFW/glfw3.h>

#include <unordered_map>
#include <string>

namespace viewer {

void SimpleWindow::key_pressed(int key, int scancode, int action, int mods) {
  if (action == GLFW_PRESS) {
    held_keys_[key] = true;
  } else if (action == GLFW_RELEASE) {
    held_keys_[key] = false;
  }

  on_key(key, scancode, action, mods);
}

void SimpleWindow::mouse_button(int button, int action, int mods) {
  if (action == GLFW_PRESS) {
    held_mouse_buttons_[button] = true;
  } else if (action == GLFW_RELEASE) {
    held_mouse_buttons_[button] = false;
  }

  on_mouse_button(button, action, mods);
}

void SimpleWindow::mouse_moved(double x, double y) {
  mouse_pos_.point(0) = x;
  mouse_pos_.point(1) = y;
  on_mouse_move(mouse_pos_);
}

bool SimpleWindow::left_mouse_held() const {
  return held_mouse_buttons_[GLFW_MOUSE_BUTTON_LEFT];
}

bool SimpleWindow::right_mouse_held() const {
  return held_mouse_buttons_[GLFW_MOUSE_BUTTON_RIGHT];
}

void SimpleWindow::resize(const GlSize &gl_size) {
  glViewport(0, 0, gl_size.width, gl_size.height);
  gl_size_ = gl_size;
}

void SimpleWindow::set_title(const std::string title) {
  title_ = title;
}

const std::string &SimpleWindow::title() const {
  return title_;
}

const std::unordered_map<int, bool> &SimpleWindow::held_keys() const {
  return held_keys_;
}

const std::array<bool, 3> &SimpleWindow::held_mouse_buttons() const {
  return held_mouse_buttons_;
}

const WindowPoint &SimpleWindow::mouse_pos() const {
  return mouse_pos_;
}
}
