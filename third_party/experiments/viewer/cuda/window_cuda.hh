#pragma once

#include "viewer/gl_size.hh"
#include "viewer/projection.hh"
#include "viewer/simple_window.hh"

#include "sophus.hh"
#include "viewer/primitives/primitive.hh"

#include "eigen.hh"

#include <atomic>
#include <memory>
#include <mutex>
#include <vector>

namespace viewer {
namespace {
using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;
}  // namespace

class WindowCuda final : public SimpleWindow {
 public:
  WindowCuda() = default;

  void on_key(int key, int scancode, int action, int mods) override;
  void on_mouse_button(int button, int action, int mods) override;
  void on_mouse_move(const WindowPoint &mouse_pos) override;
  void on_scroll(const double amount) override;

  void render() override;

  void resize(const GlSize &gl_size) override;

  void spin_until_step();

  void add_primitive(const std::shared_ptr<Primitive> primitive) {
    const std::lock_guard<std::mutex> lk(behavior_mutex_);
    primitives_.push_back(std::move(primitive));
  }

  void set_target_from_world(const SE3 &se3) {
    view_.target_from_world = se3;
  }

  void clear() {
    const std::lock_guard<std::mutex> lk(behavior_mutex_);
    primitives_.clear();
  }

 private:
  void apply_keys_to_view();

  Vec2 mouse_direction_ = Vec2::Zero();

  //
  // Track some window properties
  //

  View3D view_;
  Projection projection_;
  std::atomic<bool> should_step_{false};
  std::atomic<bool> should_continue_{false};

  GlSize gl_size_ = GlSize(640, 640);

  WindowPoint mouse_pos_last_click_;

  mutable std::mutex behavior_mutex_;
};

std::shared_ptr<WindowCuda> get_window_cuda(const std::string &title = "main");
}  // namespace viewer
