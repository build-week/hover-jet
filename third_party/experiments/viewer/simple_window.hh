#pragma once

#include "viewer/gl_size.hh"
#include "viewer/gl_types.hh"

#include <string>
#include <unordered_map>

namespace viewer {

//
// The window that you, the neighborhood dingus, get to manipulate
//
class SimpleWindow {
 public:
  virtual ~SimpleWindow() = default;
  virtual void key_pressed(int key, int scancode, int action, int mods);
  virtual void mouse_button(int button, int action, int mods);
  virtual void mouse_moved(double x, double y);

  virtual void on_key(int key, int scancode, int action, int mods) {
  }

  virtual void on_mouse_button(int button, int action, int mods) {
  }

  virtual void on_mouse_move(const WindowPoint &pos) {
  }

  virtual void on_scroll(const double amount) {
  }

  virtual void resize(const GlSize &gl_size);

  virtual void render() = 0;

  virtual void close() {
    should_close_ = true;
  };

  virtual bool should_close() const {
    return should_close_;
  }

  // Make the title something
  void set_title(const std::string title);

  // Get the title
  const std::string &title() const;

  const std::unordered_map<int, bool> &held_keys() const;
  const std::array<bool, 3> &held_mouse_buttons() const;

  const WindowPoint &mouse_pos() const;

  bool left_mouse_held() const;
  bool right_mouse_held() const;

 private:
  std::string title_;
  std::unordered_map<int, bool> held_keys_;
  std::array<bool, 3> held_mouse_buttons_ = {false, false, false};
  bool should_close_ = false;

  GlSize gl_size_;
  WindowPoint mouse_pos_;
};
}  // namespace viewer
