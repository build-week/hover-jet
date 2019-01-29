#pragma once

#include "viewer/gl_size.hh"
#include "viewer/interaction/view3d.hh"
#include "viewer/projection.hh"
#include "viewer/simple_window.hh"

#include "sophus.hh"
#include "viewer/primitives/camera.hh"
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
using Vec2Map = Eigen::Map<const Eigen::Vector2d>;
using Vec3Map = Eigen::Map<const Eigen::Vector3d>;
}  // namespace

class Window3D final : public SimpleWindow {
 public:
  Window3D(const GlSize &gl_size) : gl_size_(gl_size){};

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
    const std::lock_guard<std::mutex> lk(behavior_mutex_);
    view_.set_anchor_from_world(se3);
  }

  template <typename PrimitiveType, typename... Args>
  std::shared_ptr<PrimitiveType> add_primitive(const Args &... args) {
    const std::lock_guard<std::mutex> lk(behavior_mutex_);

    auto ptr = std::make_shared<PrimitiveType>(args...);
    primitives_.push_back(ptr);
    return ptr;
  }

  void add_camera(const std::shared_ptr<Camera> &camera) {
    cameras_.push_back(camera);
  }

  void clear() {
    const std::lock_guard<std::mutex> lk(behavior_mutex_);
    primitives_.clear();
  }

  void set_continue_time_ms(const int dt_ms) {
    continue_ms_ = dt_ms;
  }

  void set_azimuth(const double azimuth) {
    view_.set_azimuth(azimuth);
  }
  void set_elevation(const double elevation) {
    view_.set_elevation(elevation);
  }
  void set_zoom(const double zoom) {
    view_.set_zoom(zoom);
  }

  void set_orthogonal(const bool ortho = true) {
    orthogonal_projection_ = ortho;
  }

  void set_show_axes(const bool hide_axes = true) {
    hide_axes_ = hide_axes;
  }

 private:
  void draw_all_primitives() const;

  //
  // Rendering properties
  //

  Projection projection_;
  bool orthogonal_projection_ = false;
  std::vector<std::shared_ptr<Primitive>> primitives_;
  std::vector<std::shared_ptr<Camera>> cameras_;

  GlSize gl_size_;

  //
  // UI Control State
  //

  WindowPoint mouse_pos_last_click_;
  OrbitCamera view_;
  int continue_ms_ = 100;
  bool hide_axes_ = false;

  //
  // Interaction History
  //

  double last_update_time_ = 0.0;

  //
  // Threading
  //
  std::atomic<bool> should_step_{false};
  std::atomic<bool> should_continue_{false};
  mutable std::mutex behavior_mutex_;
};

std::shared_ptr<Window3D> get_window3d(const std::string &title = "main");
}  // namespace viewer
