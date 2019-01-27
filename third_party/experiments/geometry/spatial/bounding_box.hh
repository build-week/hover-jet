#pragma once

#include "geometry/ray.hh"
#include "intersection.hh"

#include "eigen.hh"

#include <limits>

namespace geometry {
namespace spatial {

// Axis aligned bounding box, with some nice geometric helpers.
template <int DIM = 3, typename Scalar = double>
class BoundingBox {
 public:
  using Vec = Eigen::Matrix<Scalar, DIM, 1>;

  // Expand to contain a point
  void expand(const Vec &point) {
    lower_ = lower_.cwiseMin(point);
    upper_ = upper_.cwiseMax(point);
  }

  // Expand to contain another axis-aligned bounding box
  void expand(const BoundingBox<DIM> &box) {
    expand(box.lower());
    expand(box.upper());
  }

  bool contains(const Vec &point) const {
    return (point.array() < upper_.array()).all() &&
           (point.array() > lower_.array()).all();
  }

  Vec center() const {
    return (upper_ + lower_) * 0.5;
  }

  double surface_area() const {
    static_assert(DIM == 3, "Dimension must be 3 because how do I higher dimensions?");
    const Vec delta = upper_ - lower_;
    double sa = 0.0;
    for (int i = 0; i < DIM; ++i) {
      sa += 2.0 * delta(i) * delta((i + 1) % DIM);
    }
    return sa;
  }

  // Compute an intersection using the slab method
  Intersection intersect(const Ray &ray) const {
    static_assert(DIM == 3, "Dimension must be 3 because rays are of dim 3 so");

    double max_t_near = std::numeric_limits<double>::lowest();
    double min_t_far = std::numeric_limits<double>::max();

    for (int i = 0; i < DIM; ++i) {
      // Note: We're really just trying to avoid denormals, in principle the operations
      // that follow are permissible for arbitrary nonzero floats
      constexpr double EPS = 1e-10;
      if (std::abs(ray.direction(i)) < EPS) {
        if (ray.origin(i) < lower_(i) || ray.origin(i) > upper_(i)) {
          return {-1.0, false};
        }
      }

      const double inv_direction_i = 1.0 / ray.direction(i);
      const double t1 = (lower_(i) - ray.origin(i)) * inv_direction_i;
      const double t2 = (upper_(i) - ray.origin(i)) * inv_direction_i;

      const double t_near = std::min(t1, t2);
      const double t_far = std::max(t1, t2);

      max_t_near = std::max(max_t_near, t_near);
      min_t_far = std::min(min_t_far, t_far);
    }

    Intersection intersection;
    if (contains(ray.origin)) {
      intersection.intersected = true;
      intersection.distance = min_t_far;
    } else {
      intersection.intersected = (max_t_near < min_t_far) && max_t_near >= 0.0;
      intersection.distance = max_t_near;
    }
    return intersection;
  }

  Eigen::Vector3d nearest_point(const Eigen::Vector3d &point) const {
    static_assert(DIM == 3, "Dimension must be 3 (I think...)");
    using Vec3 = Eigen::Vector3d;
    const Vec3 pc = point - center();
    const Vec3 b = upper() - center();
    const Vec3 abs_pt = (pc.cwiseAbs() - b).cwiseMax(Vec3::Zero());
    return point - (abs_pt.array() * pc.cwiseSign().array()).matrix();
  }

  double ud_box(const Eigen::Vector3d &point) const {
    static_assert(DIM == 3, "Dimension must be 3 (I think...)");
    using Vec3 = Eigen::Vector3d;
    const Vec3 pc = point - center();
    const Vec3 b = upper() - center();
    return (pc.cwiseAbs() - b).cwiseMax(Vec3::Zero()).norm();
  }

  const Vec &lower() const {
    return lower_;
  }

  const Vec &upper() const {
    return upper_;
  }

 private:
  Vec lower_ = Vec::Ones() * std::numeric_limits<double>::max();
  Vec upper_ = Vec::Ones() * std::numeric_limits<double>::lowest();
};
}  // namespace spatial
}  // namespace geometry
