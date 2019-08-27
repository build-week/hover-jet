//%deps(opengl, glfw, pthread, glew)

#include "visualization/lviz/cursor.hh"
#include "visualization/lviz/graphics_helpers.hh"
#include "visualization/lviz/parsing.hh"
#include "visualization/lviz/scene.hh"

#include <assert.h>
#include <math.h>
#include <any>
#include <chrono>
#include <cmath>
#include <iostream>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <variant>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <Eigen/Dense>

Scene scene;
Cursor cursor;
std::mutex buffer_modification_mutex;
std::vector<SceneUpdate> scene_element_buffer;

Eigen::Vector2d get_mouse_loc(auto window) {
  double xpos, ypos;
  glfwGetCursorPos(window, &xpos, &ypos);
  return Eigen::Vector2d(xpos, ypos);
}

std::pair<int, int> get_window_dimensions(auto window) {
  int window_width, window_height;
  glfwGetWindowSize(window, &window_width, &window_height);
  return std::make_pair(window_width, window_height);
}

void reset_viewport_size(auto window) {
  auto window_dimensions = get_window_dimensions(window);
  glViewport(0, 0, window_dimensions.first, window_dimensions.second);
}

void mouse_button_callback(GLFWwindow *window, int button, int action, int mods) {
  if (button == GLFW_MOUSE_BUTTON_RIGHT) cursor.right_mouse_button_down = action == GLFW_PRESS;
  if (button == GLFW_MOUSE_BUTTON_LEFT) cursor.left_mouse_button_down = action == GLFW_PRESS;
}

static void key_callback(GLFWwindow *window, int key, int scancode, int action, int mods) {
  if (action == GLFW_PRESS) {
    if (key == GLFW_KEY_ESCAPE) glfwSetWindowShouldClose(window, GLFW_TRUE);
    if (key == GLFW_KEY_RIGHT) scene.add_to_frame_index(1);
    if (key == GLFW_KEY_LEFT) scene.add_to_frame_index(-1);
    if (key == GLFW_KEY_SPACE) scene.toggle_is_playing();

    // set camera to look down an axis
    if (key == GLFW_KEY_X) cursor.set_camera_translation_from_cursor_normalized(Eigen::Vector3f(1, 0, 0).normalized());
    if (key == GLFW_KEY_Y)
      cursor.set_camera_translation_from_cursor_normalized(Eigen::Vector3f(0.00001, 1, 0).normalized());
    if (key == GLFW_KEY_Z) cursor.set_camera_translation_from_cursor_normalized(Eigen::Vector3f(0, 0, 1).normalized());

    // set camera to isometric view
    if (key == GLFW_KEY_I) {
      cursor.set_camera_translation_from_cursor_normalized(Eigen::Vector3f(1, 1, 1).normalized());
      cursor.perspective_mode = false;
    }

    // toggle orthognal projection
    if (key == GLFW_KEY_O) cursor.perspective_mode = !cursor.perspective_mode;
  }
}

void scroll_callback(GLFWwindow *window, double xoffset, double yoffset) {
  cursor.camera_distance_from_cursor += yoffset;
  cursor.camera_distance_from_cursor = std::clamp(cursor.camera_distance_from_cursor, 1.0, 10000.0);
}

void run_stream_parser() {
  // vastly increases stream reading speed
  std::ios_base::sync_with_stdio(false);
  std::string line;
  while (true) {
    while (std::getline(std::cin, line)) {
      // while(true){
      // if (line.length() > 0) {
      auto new_element = scene_element_from_line(line);
      if (new_element.has_value()) {
        buffer_modification_mutex.lock();
        scene_element_buffer.push_back(new_element.value());
        buffer_modification_mutex.unlock();
      }
      // }
    }
    // TODO replace with e.g. interrupt
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

void apply_scene_changes(Scene &scene) {
  buffer_modification_mutex.lock();

  for (auto element : scene_element_buffer) {
    if (std::holds_alternative<ColoredPoint>(element)) {
      scene.add_point(std::get<ColoredPoint>(element));
    } else if (std::holds_alternative<std::pair<ColoredPoint, ColoredPoint>>(element)) {
      scene.add_line(std::get<std::pair<ColoredPoint, ColoredPoint>>(element));
    } else if (std::holds_alternative<NewScene>(element)) {
      scene.add_frame();
    } else if (std::holds_alternative<SetFrameTime>(element)) {
      scene.set_frame_duration(std::get<SetFrameTime>(element).frame_duration);
    }
  }
  std::cout << scene_element_buffer.size() << std::endl;
  scene_element_buffer.clear();
  buffer_modification_mutex.unlock();
}

int main() {
  auto window = get_window();
  glfwSetMouseButtonCallback(window, mouse_button_callback);
  glfwSetScrollCallback(window, scroll_callback);
  glfwSetKeyCallback(window, key_callback);
  GLuint shader_program_id = LoadShaders();
  scene.setup_buffers();
  glUseProgram(shader_program_id);

  int frames_so_far = 0;
  cursor.last_mouse_loc = get_mouse_loc(window);
  cursor.current_mouse_loc = get_mouse_loc(window);

  std::thread parsing_thread(run_stream_parser);

  auto frame_duration = std::chrono::milliseconds(250);
  auto frame_time_remaining = frame_duration;

  while (!glfwWindowShouldClose(window)) {
    auto render_start_time = std::chrono::high_resolution_clock::now();

    // update mouse history
    // TODO own method
    auto mouse_delta = cursor.current_mouse_loc - cursor.last_mouse_loc;
    cursor.last_mouse_loc = cursor.current_mouse_loc;
    cursor.current_mouse_loc = get_mouse_loc(window);

    frames_so_far++;

    // TODO make function
    // calculate mouse movements
    const Eigen::Vector4f mouse_move_image_space(-mouse_delta.x(), mouse_delta.y(), 0.0, 0.0);
    cursor.update_cursor_state(mouse_move_image_space, cursor.right_mouse_button_down, cursor.left_mouse_button_down,
                               cursor.camera_distance_from_cursor);

    // TODO make function
    auto window_dimensions = get_window_dimensions(window);
    auto aspect_ratio = 1.0 * window_dimensions.first / window_dimensions.second;
    Eigen::Matrix4f image_from_camera =
        get_image_from_view(aspect_ratio, .01, 10000, cursor.camera_distance_from_cursor, !cursor.perspective_mode);
    Eigen::Matrix4f image_from_world = image_from_camera * cursor.camera_from_world(cursor.camera_distance_from_cursor);

    GLuint image_from_world_idx = glGetUniformLocation(shader_program_id, "image_from_world");
    glUniformMatrix4fv(image_from_world_idx, 1, GL_FALSE, image_from_world.data());

    reset_viewport_size(window);

    // Keep running
    glfwSwapBuffers(window);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearDepth(0.0f);
    glClearColor(0, 43.0f / 255, 54.0f / 255, 1.0f);  // solarized base03

    glfwPollEvents();
    scene.draw();
    apply_scene_changes(scene);
    auto render_end_time = std::chrono::high_resolution_clock::now();
    auto loop_time_so_far = std::chrono::duration_cast<std::chrono::milliseconds>(render_end_time - render_start_time);
    frame_time_remaining = frame_time_remaining - loop_time_so_far;
    if ((frame_time_remaining.count() < 0) && scene.get_is_playing()) {
      scene.add_to_frame_index(1);
      frame_time_remaining = std::chrono::milliseconds((int)(scene.get_current_frame().display_duration * 1000));
    }
  }
  glfwDestroyWindow(window);
  glfwTerminate();

  return 0;
}
