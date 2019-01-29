#include "viewer/projection.hh"

//%deps(opengl)

#include <GL/glew.h>
#include <GLFW/glfw3.h>

namespace viewer {

// (Static)
Projection Projection::get_from_current() {
  //
  // Get the projection matrix
  //

  double projection[16];
  glGetDoublev(GL_PROJECTION_MATRIX, projection);
  const Eigen::Map<Mat4, Eigen::RowMajor> gl_projection_mat(projection);
  const Mat4                              eigen_projection = gl_projection_mat;

  //
  // Get a the modelview matrix
  //

  double modelview[16];
  glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
  const Eigen::Map<Mat4, Eigen::RowMajor> gl_modelview_mat(modelview);
  const Mat4                              eigen_modelview = gl_modelview_mat;

  //
  // Get the viewport dimensions
  //

  GLint viewport_dimensions[4];
  glGetIntegerv(GL_VIEWPORT, viewport_dimensions);
  const Eigen::Map<Vec4i> gl_viewport_dimensions(viewport_dimensions);
  const Vec4i             eigen_viewport_dimension = gl_viewport_dimensions;

  return Projection(eigen_projection, eigen_modelview, eigen_viewport_dimension);
}

ViewportPoint Projection::to_viewport(const WindowPoint& window_point) const {
  // Return a viewport point!

  const double width  = viewport_dimensions_.cast<double>()(2);
  const double height = viewport_dimensions_.cast<double>()(3);

  const double x = (2.0f * window_point.point.x()) / width - 1.0f;
  const double y = 1.0f - (2.0f * window_point.point.y()) / height;

  const Vec2          view_point_eigen(x, y);
  const ViewportPoint view_point(view_point_eigen);

  return view_point;
}

geometry::Ray Projection::unproject(const WindowPoint& window_point) const {
  const ViewportPoint viewport_pt = to_viewport(window_point);
  return unproject(viewport_pt);
}

geometry::Ray Projection::unproject(const ViewportPoint& view_point) const {
  const Vec4 view_point_h_near = Vec4(view_point.point(0), view_point.point(1), 0.0, 1.0);
  const Vec4 view_point_h_far  = Vec4(view_point.point(0), view_point.point(1), 1.0, 1.0);

  const Vec4 p_h_near = lu_.solve(view_point_h_near);
  const Vec4 p_h_far  = lu_.solve(view_point_h_far);

  const Vec3 p_near = Vec3(p_h_near(0), p_h_near(1), p_h_near(2)) / p_h_near(3);
  const Vec3 p_far  = Vec3(p_h_far(0), p_h_far(1), p_h_far(2)) / p_h_far(3);

  return geometry::Ray({p_near, (p_far - p_near).normalized()});
}

ViewportPoint Projection::project(const Vec3& world_point) const {
  const Vec4 world_point_h = (Vec4() << world_point, 1.0).finished();

  // todo: don't repeat this multiply
  const Vec4 view_point_h = projection_mat_ * (modelview_mat_ * world_point_h);

  const ViewportPoint point(view_point_h.head<2>() / view_point_h(3));
  return point;
}

}  // namespace viewer
