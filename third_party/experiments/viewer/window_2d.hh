#pragma once

#include "viewer/gl_size.hh"
#include "viewer/projection.hh"
#include "viewer/simple_window.hh"

#include "viewer/primitives/primitive.hh"
#include "viewer/primitives/primitive_types.hh"

#include "eigen.hh"
#include <sophus/se2.hpp>

#include <memory>
#include <mutex>
#include <vector>

namespace viewer {
namespace {
using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;
using Vec2Map = Eigen::Map<const Eigen::Vector2d>;
using Vec3Map = Eigen::Map<const Eigen::Vector3d>;
}  // namespace

class Window2D final : public SimpleWindow {
  using so2 = Sophus::SO2<double>;
  using se2 = Sophus::SE2<double>;

 public:
  Window2D() {
    view_.camera_height = 3.0;
  }

  void on_key(int key, int scancode, int action, int mods) override;
  void on_mouse_button(int button, int action, int mods) override;
  void on_mouse_move(const WindowPoint &mouse_pos) override;
  void on_scroll(const double amount) override;

  void resize(const GlSize &gl_size) override;

  void render() override;

  void spin_until_step();

  void add_line(const Line &line) {
    std::lock_guard<std::mutex> lk(behavior_mutex_);
    renderables_.lines.push_back(line);
  }

  void add_ray(const Ray &ray) {
    std::lock_guard<std::mutex> lk(behavior_mutex_);
    renderables_.rays.push_back(ray);
  }

  void add_circle(const Circle &circle) {
    std::lock_guard<std::mutex> lk(behavior_mutex_);
    renderables_.circles.push_back(circle);
  }

  void add_points(const Points &points) {
    std::lock_guard<std::mutex> lk(behavior_mutex_);
    renderables_.points.push_back(points);
  }

  void add_primitive(const std::shared_ptr<Primitive> primitive) {
    std::lock_guard<std::mutex> lk(behavior_mutex_);
    primitives_.push_back(std::move(primitive));
  }

  void clear() {
    std::lock_guard<std::mutex> lk(behavior_mutex_);

    primitives_.clear();
    renderables_.clear();
  }

 private:
  struct View2D {
    // Default identity
    se2 camera_pose;

    // Height from view plane
    double camera_height = 5.0;
    double dcamera_height = 0.0;

    double zoom = 0.1;
    double dzoom = 0.0;

    // Time
    double last_update_time = 0.0;

    // Tangent vector; Left tangent space or gtfo
    Vec3 velocity = Vec3::Zero();

    // Apply the transformation
    void apply();

    void simulate();
  };

  void apply_keys_to_view();

  void draw_renderables(const Renderables &renderables) const;

  Vec2 mouse_direction_ = Vec2::Zero();

  //
  // Track some window properties
  //

  View2D view_;
  Projection projection_;
  bool should_continue_ = false;

  Renderables renderables_;

  std::vector<std::shared_ptr<Primitive>> primitives_;

  GlSize gl_size_ = GlSize(640, 640);
  mutable std::mutex behavior_mutex_;
};

std::shared_ptr<Window2D> get_window2d(const std::string &title = "main");
}  // namespace viewer
