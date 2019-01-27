#pragma once
//%deps(opencv, opengl)

#include "viewer/gl_renderable_buffer.hh"

#include "sophus.hh"
#include "viewer/projection.hh"

#include <mutex>
#include <opencv2/opencv.hpp>

namespace viewer {

class Camera final {
 public:
  Camera(const GlSize& size = GlSize(640, 640)) : size_(size) {
  }

  void draw();

  void set_world_from_camera(const SE3& world_from_camera) {
    const std::lock_guard<std::mutex> lk(draw_mutex_);
    world_from_camera_ = world_from_camera;
    have_image_ = false;
  }

  GlSize size() const {
    return size_;
  }

  Projection get_projection() const {
    while (!have_image()) {
    }
    return proj_;
  }

  cv::Mat extract_image() const {
    while (!have_image()) {
    }
    const std::lock_guard<std::mutex> lk(draw_mutex_);
    return image_;
  }

  bool have_image() const {
    const std::lock_guard<std::mutex> lk(draw_mutex_);
    return have_image_;
  }

  void prepare_view() const;

 private:
  mutable std::mutex draw_mutex_;

  cv::Mat capture_framebuffer() const;

  Projection proj_;
  GlSize size_;

  SE3 world_from_camera_;
  bool have_image_ = false;
  cv::Mat image_;
};
}  // namespace viewer
