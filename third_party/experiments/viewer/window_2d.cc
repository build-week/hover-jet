// jcc is licensed under the commonwealth of the borg act
// All members of the collective may share in its bounty
//

//%deps(opengl, glfw)

#include "viewer/window_2d.hh"

// inc order weird for these guys
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "geometry/plane.hh"
#include "viewer/gl_aliases.hh"
#include "viewer/window_manager.hh"

#include <iostream>
#include <thread>
#include <map>

namespace viewer {
namespace {
void draw_points(const Points &points) {
  glColor(points.color);
  glBegin(GL_POINTS);
  {
    for (const auto &pt : points.points) {
      glVertex(pt);
    }
  }
  glEnd();
}
void pre_render() {
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

  glEnable(GL_SAMPLE_ALPHA_TO_COVERAGE);
  glEnable(GL_LINE_SMOOTH);
  glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
}
}  // namespace

void Window2D::View2D::apply() {
  //
  // Projection
  //

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  glTranslated(0.0, 0.0, -camera_height);
  glScaled(zoom, zoom, zoom);
  glRotate(camera_pose.so2().inverse());
  glTranslate((-camera_pose.translation()).eval());

  simulate();
}

void Window2D::View2D::simulate() {
  //
  // Deal with time
  //

  const double t_now = glfwGetTime();
  const double dt = t_now - last_update_time;
  last_update_time = t_now;

  //
  // Apply the current transform and derivatives
  //

  const Vec3 delta = dt * velocity;
  const se2 expdelta = se2::exp(delta);
  camera_pose = expdelta * camera_pose;

  camera_height += dcamera_height * dt;
  zoom *= std::exp(0.2 * dzoom * dt);

  //
  // Apply damping
  //

  constexpr double translation_damping = 0.95;
  constexpr double rotation_damping = 0.98;
  constexpr double scroll_damping = 0.90;
  constexpr double zoom_damping = 0.85;

  velocity.head<2>() *= translation_damping;
  velocity(2) *= rotation_damping;

  dcamera_height *= scroll_damping;
  dzoom *= zoom_damping;
}

void Window2D::on_key(int key, int scancode, int action, int mods) {
  if (action == GLFW_PRESS) {
    if (key == static_cast<int>('N')) {
      should_continue_ = true;
    }
  }
}

void Window2D::spin_until_step() {
  while (!should_continue_) {
    // std::lock_guard<std::mutex> lk(behavior_mutex_);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  should_continue_ = false;
}

bool point_on_plane(const Projection &proj, const WindowPoint &point, Out<Vec3> intersection) {
  const geometry::Ray ray = proj.unproject(point);
  const geometry::Plane plane({Vec3::Zero(), Vec3::UnitZ()});
  return plane.intersect(ray, intersection);
}

void Window2D::on_mouse_button(int button, int action, int mods) {
  std::lock_guard<std::mutex> lk(behavior_mutex_);
  if (left_mouse_held()) {
  }
}

void Window2D::on_mouse_move(const WindowPoint &position) {
  std::lock_guard<std::mutex> lk(behavior_mutex_);
  if (left_mouse_held()) {
  }

  if (right_mouse_held()) {
    Vec3 intersection;
    if (point_on_plane(projection_, position, out(intersection))) {
      const double t_now = glfwGetTime();
      const Vec4 color((std::sin(t_now / 5.0) + 1.0) * 0.5, (std::sin(t_now / 1.0) + 1.0) * 0.5, 0.2, 0.9);
      (void)color;
      // add_circle({Vec2(intersection.x(), intersection.y()), 0.3, color});
    }
  }
}

void Window2D::on_scroll(const double amount) {
  std::lock_guard<std::mutex> lk(behavior_mutex_);
  const double scroll_acceleration = 0.8;
  view_.dzoom += amount * scroll_acceleration;
}

void Window2D::resize(const GlSize &gl_size) {
  std::lock_guard<std::mutex> lk(behavior_mutex_);
  glViewport(0, 0, gl_size.width, gl_size.height);
  gl_size_ = gl_size;
}

void Window2D::apply_keys_to_view() {
  const auto keys = held_keys();

  const double acceleration = 0.1 / view_.zoom;

  Vec3 delta_vel = Vec3::Zero();
  for (const auto &key_element : keys) {
    const bool held = key_element.second;
    const int key = key_element.first;

    if (!held) {
      continue;
    }

    switch (key) {
      case (static_cast<int>('W')):
        delta_vel(1) += acceleration;
        break;

      case (static_cast<int>('A')):
        delta_vel(0) -= acceleration;
        break;

      case (static_cast<int>('S')):
        delta_vel(1) -= acceleration;

        break;
      case (static_cast<int>('D')):
        delta_vel(0) += acceleration;
        break;
    }
  }

  view_.velocity += delta_vel;
}

void Window2D::draw_renderables(const Renderables &renderables) const {
  //
  // Draw lines
  //

  glBegin(GL_LINES);
  for (const auto &line : renderables.lines) {
    glColor(line.color);
    glVertex(line.start);
    glVertex(line.end);
  }
  glEnd();

  //
  // Draw rays
  //

  glBegin(GL_LINES);
  for (const auto &ray : renderables.rays) {
    glColor(ray.color);
    glVertex(ray.origin);

    const Vec4 v4 = (Vec4() << ray.direction, 0.0, 0.0).finished();
    glVertex(v4);
  }
  glEnd();

  //
  // Draw circles
  //

  for (const auto &circle : renderables.circles) {
    constexpr double SEGMENTS_PER_RADIAN = 20.0;
    const double circumference = circle.radius * 2.0 * M_PI;
    const int num_segments = static_cast<int>(SEGMENTS_PER_RADIAN * circumference);
    const double segment_angular_fraction = 2.0 * M_PI / num_segments;

    glColor(circle.color);
    // glBegin(GL_LINE_LOOP);
    glBegin(GL_LINES);
    for (double angular_fraction = 0.0; angular_fraction < (2.0 * M_PI); angular_fraction += segment_angular_fraction) {
      const double x = (circle.radius * std::cos(angular_fraction)) + circle.center.x();
      const double y = (circle.radius * std::sin(angular_fraction)) + circle.center.y();

      glVertex2d(x, y);
    }
    glEnd();
  }

  //
  // Draw points
  //

  for (const auto &pts : renderables.points) {
    draw_points(pts);
  }
}

void Window2D::render() {
  std::lock_guard<std::mutex> lk(behavior_mutex_);
  pre_render();

  //
  // Apply zoom
  //

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(60, static_cast<double>(gl_size_.width) / static_cast<double>(gl_size_.height), 0.1, 100.0);

  //
  // Compute ground plane intersection
  //

  // Update projection
  projection_ = Projection::get_from_current();

  //
  // Draw all renderables
  //

  apply_keys_to_view();
  view_.apply();

  for (const auto &primitive : primitives_) {
    primitive->draw();
  }

  draw_renderables(renderables_);

  glFlush();
  glFinish();
}

struct Window2DGlobalState {
  std::map<std::string, std::shared_ptr<Window2D>> windows;
};

Window2DGlobalState window_2d_state;

std::shared_ptr<Window2D> get_window2d(const std::string &title) {
  const auto it = window_2d_state.windows.find(title);
  if (it != window_2d_state.windows.end()) {
    return it->second;
  } else {
    auto window = std::make_shared<Window2D>();
    window_2d_state.windows[title] = window;
    WindowManager::register_window(GlSize(640, 640), window, title);
    return window;
  }
}
}  // namespace viewer
