#pragma once

#include "viewer/gl_types.hh"
#include "geometry/ray.hh"

#include "eigen.hh"

#include <stdexcept>

namespace viewer {

class Projection {
  using Mat4 = Eigen::Matrix4d;

  using Vec2 = Eigen::Vector2d;
  using Vec3 = Eigen::Vector3d;
  using Vec4 = Eigen::Vector4d;

  using Vec4i = Eigen::Vector4i;

 public:
  static Projection get_from_current();

  Projection() = default;

  Projection(const Mat4& projection_matrix, const Mat4& modelview_matrix, const Vec4i& viewport_dimensions)
      : valid_(true),
        projection_mat_(projection_matrix),
        modelview_mat_(modelview_matrix),
        viewport_dimensions_(viewport_dimensions) {
    lu_ = Eigen::FullPivLU<Mat4>(projection_mat_ * modelview_mat_);

    // SORRY YO
    if (lu_.rank() != 4) {
      throw std::runtime_error("Can't invert projection-model-view matrix, failing");
    }
  }

  // Get a view point from a window point
  //
  ViewportPoint to_viewport(const WindowPoint& window_point) const;

  // Get a ray for a window point
  //
  geometry::Ray unproject(const WindowPoint& window_point) const;

  // Get a ray for a view point
  //
  geometry::Ray unproject(const ViewportPoint& view_point) const;

  // Get a view point for a 3d point
  //
  ViewportPoint project(const Vec3& world_point) const;

  const Mat4& projection_mat() const {
    return projection_mat_;
  }

  const Mat4& modelview_mat() const {
    return modelview_mat_;
  }

 private:
  bool valid_ = false;

  Eigen::FullPivLU<Mat4> lu_;
  Mat4                   projection_mat_;
  Mat4                   modelview_mat_;
  Vec4i                  viewport_dimensions_;
};

}  // namespace viewer
