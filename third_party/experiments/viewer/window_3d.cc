// inc order weird for these guys
#include <GL/glew.h>
#include <GLFW/glfw3.h>

//%deps(opengl, glfw)

#include "viewer/gl_aliases.hh"
#include "viewer/primitives/simple_geometry_primitives.hh"
#include "viewer/window_3d.hh"
#include "viewer/window_manager.hh"

#include "eigen_helpers.hh"

#include <map>
#include <thread>

namespace viewer {
namespace {

void apply_view(const OrbitCamera &view, bool show_axes) {
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  glTransform(view.camera_from_anchor());
  glScaled(view.zoom(), view.zoom(), view.zoom());
  if (show_axes) {
    draw_axes({SE3(), 0.1 / view.zoom(), 3.0});
  }

  glTransform(view.anchor_from_world());
  if (show_axes) {
    draw_axes({SE3(), 0.2 / view.zoom(), 3.0});
  }
}

void prepare_to_render() {
  //
  // Flag soup
  //

  glShadeModel(GL_SMOOTH);

  // Check depth when rendering
  glEnable(GL_DEPTH_TEST);

  // Turn on lighting
  // glEnable(GL_LIGHTING);

  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_BLEND);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // glEnable(GL_SAMPLE_ALPHA_TO_COVERAGE);
  glEnable(GL_LINE_SMOOTH);
  glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);

  glEnable(GL_MULTISAMPLE);
}

void set_perspective(const GlSize &gl_size, bool ortho = false) {
  glViewport(0, 0, gl_size.width, gl_size.height);
  glMatrixMode(GL_PROJECTION);

  glLoadIdentity();

  constexpr double NEAR_CLIP = 0.001;
  constexpr double FAR_CLIP = 1000.0;
  const double aspect_ratio =
      (static_cast<double>(gl_size.width) / static_cast<double>(gl_size.height));
  if (ortho) {
    glOrtho(-aspect_ratio, aspect_ratio, -1.0, 1.0, NEAR_CLIP, FAR_CLIP);
  } else {
    constexpr double FOV = 60.0;
    gluPerspective(FOV, aspect_ratio, NEAR_CLIP, FAR_CLIP);
  }
}

}  // namespace

void Window3D::spin_until_step() {
  while (!should_step_ && !should_continue_ && !should_close()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  if (should_continue_ && !should_close()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(continue_ms_));
  }

  if (should_step_) {
    should_step_ = false;
  }
}

void Window3D::on_key(int key, int scancode, int action, int mods) {
  const std::lock_guard<std::mutex> lk(behavior_mutex_);

  if (action == GLFW_PRESS) {
    if (key == static_cast<int>('N')) {
      should_step_ = true;
    }

    if (key == static_cast<int>('C')) {
      should_continue_ = !should_continue_;
    }

    if (key == static_cast<int>('O')) {
      orthogonal_projection_ = !orthogonal_projection_;
    }
    if (key == static_cast<int>('H')) {
      hide_axes_ = !hide_axes_;
    }
  }
}
void Window3D::on_mouse_button(int button, int action, int mods) {
  const std::lock_guard<std::mutex> lk(behavior_mutex_);

  mouse_pos_last_click_ = mouse_pos();
}

void Window3D::on_mouse_move(const WindowPoint &mouse_pos) {
  const std::lock_guard<std::mutex> lk(behavior_mutex_);

  const bool shift = held_keys().count(GLFW_KEY_LEFT_SHIFT) == 1
                         ? held_keys().at(GLFW_KEY_LEFT_SHIFT)
                         : false;
  const bool left = left_mouse_held() && !shift;
  const bool right = right_mouse_held() || (shift && left_mouse_held());
  view_.apply_mouse(mouse_pos, mouse_pos_last_click_, left, right);

  if (left || right) {
    mouse_pos_last_click_ = mouse_pos;
  }
}

void Window3D::on_scroll(const double amount) {
  const std::lock_guard<std::mutex> lk(behavior_mutex_);
  view_.apply_scroll(amount);
}

void Window3D::resize(const GlSize &gl_size) {
  const std::lock_guard<std::mutex> lk(behavior_mutex_);

  glViewport(0, 0, gl_size.width, gl_size.height);
  gl_size_ = gl_size;
}

void Window3D::draw_all_primitives() const {
  for (const auto &primitive : primitives_) {
    primitive->draw();
  }
}

void Window3D::render() {
  const std::lock_guard<std::mutex> lk(behavior_mutex_);

  //
  // Render cameras
  //
  for (const auto &camera : cameras_) {
    prepare_to_render();
    camera->prepare_view();
    draw_all_primitives();
    camera->draw();
  }

  {  // Render main scene
    set_perspective(gl_size_, orthogonal_projection_);
    prepare_to_render();
    apply_view(view_, !hide_axes_);
    projection_ = Projection::get_from_current();
    draw_all_primitives();
  }

  const double t_now = glfwGetTime();

  const double dt = std::max(t_now - last_update_time_, 0.02);
  view_.apply_keys(held_keys(), dt);
  view_ = view_.simulate(dt);
  last_update_time_ = t_now;

  glFinish();
}  // namespace viewer

struct Window3DGlobalState {
  std::map<std::string, std::shared_ptr<Window3D>> windows;
};

Window3DGlobalState window_3d_state;

std::shared_ptr<Window3D> get_window3d(const std::string &title) {
  const auto it = window_3d_state.windows.find(title);
  if (it != window_3d_state.windows.end()) {
    return it->second;
  } else {
    const GlSize gl_size(640, 640);
    auto window = std::make_shared<Window3D>(gl_size);
    window_3d_state.windows[title] = window;
    WindowManager::register_window(gl_size, window, title);
    return window;
  }
}
}  // namespace viewer
