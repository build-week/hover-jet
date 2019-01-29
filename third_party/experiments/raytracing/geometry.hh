#pragma once

#include "eigen.hh"

#include "out.hh"

namespace raytrace {

namespace {
using Vec2 = Eigen::Vector2d;
}

double cross2d(const Vec2 &a, const Vec2 &b);

// A proper geometric ray, that points endlessly in `direction`
//
//
struct Ray {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Ray(const Vec2 &origin_, const Vec2 &direction_) : origin(origin_), direction(direction_.normalized()) {
  }
  Vec2 origin;
  Vec2 direction;
};

// A proper geometric line, that points endlessly in both `direction` and `-direction`
//
//
struct Line {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Line(const Vec2 &point_, const Vec2 &direction_) : point(point_), direction(direction_.normalized()) {
  }
  Vec2 point;
  Vec2 direction;
};

// A 2D line segment (parameterized by `start` and `end`)
//
//
struct LineSegment {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LineSegment(const Vec2 &start_, const Vec2 &end_) : start(start_), end(end_) {
  }
  Vec2 start;
  Vec2 end;
};

// A 2D plane section (parameterized by `normal` and `radius`)
// Equivalent to a line segment, but with a "sidedness", determined by `normal`
//
struct DirectedLineSegment {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  DirectedLineSegment(const Vec2 &normal_, const Vec2 &center_, const double radius_)
      : normal(normal_.normalized()), center(center_), radius(radius_) {
  }
  DirectedLineSegment(const Vec2 &normal_, const Vec2 &center_)
      : normal(normal_.normalized()), center(center_), radius(-1.0) {
  }

  Vec2   normal;
  Vec2   center;
  double radius;

  // Signed
  // `point`s in the halfspace indicated by `normal` will have positive distance
  double to_plane_distance(const Vec2 &point) const {
    // Project into the normal subspace
    return normal.dot(point - center);
  }

  Vec2 dto_plane_distance(const Vec2 &point) const {
    return normal;
  }

  // Signed
  // `point`s in the halfspace indicated by `normal` will have positive distance
  double along_plane_distance(const Vec2 &point) const {
    // Norm of the vector projected into the direction subspace
    return cross2d(normal, point - center);
  }

  Vec2 dalong_plane_distance(const Vec2 &point) const {
    return Vec2(-normal(1), normal(0));
  }
};

// Compute the intersection of a line and a ray
//
// Parameters
// ----------
// @param ray The ray
// @param line The line to be intersected
//
// Outputs
// -------
// @param[out] intersection The point at which the ray and line intersect
//                          Note: If the ray is not pointed at the line, populated with the point
//                                that would have been the intersection if the ray were reversed
//
// @returns False if the ray does not intersect the line
//
bool ray_line_intersection(const Ray &ray, const Line &line, Out<Vec2> intersection);

// Compute the intersection of a line segment and a ray
//
// Parameters
// ----------
// @param ray The ray
// @param segment The line segment to be intersected
//
// Outputs
// -------
// @param[out] intersection The point at which the ray and line segment intersect
//                          Note: If the ray is not pointed at the segment, populated with the point
//                                that would have been the intersection if the ray were reversed
//
//                          Furthermore: If the ray *misses* the line segment, returns the
//                                       point where the infinitely extended line segment
//                                       would have intersected the ray-equivalent line
//
// @returns False if the ray does not intersect the line segment
//
bool ray_line_segment_intersection(const Ray &ray, const LineSegment &segment, Out<Vec2> intersection);
}
