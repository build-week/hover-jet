#pragma once

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "viewer/gl_size.hh"
#include "viewer/simple_window.hh"  // Can't include before glew

#include <memory>

namespace viewer {

//
// Callback router
//
class WindowManager {
 public:
  // Tool for managing windows

  // Create and register windows with the manager
  //
  static void register_window(const GlSize& size,
                              const std::shared_ptr<SimpleWindow> win,
                              const std::string& window_name,
                              int win_ver_maj = 2);

  // Render all of the managed windows
  //
  static void render();

  // Are there any active windows?
  //
  static bool any_windows();

  // Block and run until an exit is requested
  static void spin();

  static void draw(const int ms = 16);
};
}  // namespace viewer