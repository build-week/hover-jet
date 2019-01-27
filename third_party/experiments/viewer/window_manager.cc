#include "viewer/window_manager.hh"

//%deps(OPENGL, GLFW, GLEW, pthread)

#include <atomic>
#include <chrono>
#include <iostream>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

namespace viewer {

void key_callback(GLFWwindow *window, int key, int scancode, int action, int mods);
void cursor_position_callback(GLFWwindow *window, double xpos, double ypos);
void mouse_button_callback(GLFWwindow *window, int button, int action, int mods);
void window_size_callback(GLFWwindow *window, int width, int height);
void scroll_callback(GLFWwindow *window, double xoffset, double yoffset);

void error_callback(int error, const char *description);

//
// Manage a global registry of the created windows
//

std::mutex global_state_mutex;

struct GlobalState {
  std::map<GLFWwindow *, std::shared_ptr<SimpleWindow>> windows;
  int last_placed_window_x = 0;
};
// Global state singleton
std::shared_ptr<GlobalState> global_state;

std::atomic<bool> ready(false);

void render_func() {
  while (true) {
    if (ready) {
      const std::lock_guard<std::mutex> lk(global_state_mutex);
      WindowManager::draw(16);
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }
  }
}

std::thread render_thread(render_func);

std::shared_ptr<GlobalState> maybe_create_global_state() {
  if (!global_state) {
    const std::lock_guard<std::mutex> lk(global_state_mutex);
    glfwSetErrorCallback(error_callback);

    if (!glfwInit()) {
      exit(EXIT_FAILURE);
    }

    glewExperimental = GL_TRUE;

    global_state = std::make_shared<GlobalState>();

    ready = true;
    render_thread.detach();
  }
  return global_state;
}

void WindowManager::register_window(const GlSize &size,
                                    const std::shared_ptr<SimpleWindow> simple_window,
                                    const std::string &window_name,
                                    const int win_ver_maj) {
  maybe_create_global_state();

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, win_ver_maj);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
  glfwWindowHint(GLFW_RESIZABLE, GL_TRUE);

  // 8x MSAA
  glfwWindowHint(GLFW_SAMPLES, 8);

  GLFWwindow *window =
      glfwCreateWindow(size.height, size.width, window_name.c_str(), nullptr, nullptr);
  simple_window->set_title(window_name);


  if (!window) {
    glfwTerminate();
    std::cerr << "\nFailed to create new window" << std::endl;
    exit(EXIT_FAILURE);
  }

  glfwSetWindowPos(window, global_state->last_placed_window_x, 0);

  glfwSetKeyCallback(window, key_callback);
  glfwSetCursorPosCallback(window, cursor_position_callback);
  glfwSetMouseButtonCallback(window, mouse_button_callback);
  glfwSetWindowSizeCallback(window, window_size_callback);
  glfwSetScrollCallback(window, scroll_callback);

  global_state->windows[window] = simple_window;
  global_state->last_placed_window_x += size.width;
}

//
// Render all of the managed windows
//
void WindowManager::render() {
  for (auto it = global_state->windows.begin(); it != global_state->windows.end(); it++) {
    auto &glfw_win = it->first;
    auto &window = it->second;

    glfwMakeContextCurrent(glfw_win);
    glewInit();

    if (!glfwWindowShouldClose(glfw_win)) {
      window->render();
      glfwSwapBuffers(glfw_win);
    } else {
      glfwDestroyWindow(glfw_win);
      global_state->windows.erase(it);
      continue;
    }
  }
  glfwPollEvents();
}

bool WindowManager::any_windows() {
  // Not thread-safe, figure out later
  if (global_state) {
    return !global_state->windows.empty();
  }
  return false;
}

void WindowManager::draw(const int ms) {
  int ms_slept = 0;
  while (any_windows() && (ms_slept < ms)) {
    constexpr int SLEEP_MS = 2;
    render();
    std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_MS));
    ms_slept += SLEEP_MS;
  }
}

void WindowManager::spin() {
  while (any_windows()) {
    render();
    std::this_thread::sleep_for(std::chrono::milliseconds(16));
  }
  glfwTerminate();
}

void error_callback(int error, const char *description) {
  fputs(description, stderr);
}

void close_window(GLFWwindow *window) {
  global_state->windows.at(window)->close();
}

void key_callback(GLFWwindow *window, int key, int scancode, int action, int mods) {
  if ((key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) || (key == GLFW_KEY_Q)) {
    close_window(window);
    glfwSetWindowShouldClose(window, GL_TRUE);
  }

  global_state->windows.at(window)->key_pressed(key, scancode, action, mods);
}

void cursor_position_callback(GLFWwindow *window, double xpos, double ypos) {
  global_state->windows.at(window)->mouse_moved(xpos, ypos);
}

void mouse_button_callback(GLFWwindow *window, int button, int action, int mods) {
  if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS) {
  }

  if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
  }

  if (action == GLFW_RELEASE) {
  }

  global_state->windows.at(window)->mouse_button(button, action, mods);
}

void scroll_callback(GLFWwindow *window, double xoffset, double yoffset) {
  global_state->windows.at(window)->on_scroll(yoffset);
}

void window_size_callback(GLFWwindow *window, int width, int height) {
  global_state->windows.at(window)->resize(GlSize(width, height));
}
}  // namespace viewer
