#include "geometry.hh"

namespace raytrace {

using Vec2 = Eigen::Vector2d;

double cross2d(const Vec2 &a, const Vec2 &b) {
  return (a(0) * b(1)) - (a(1) * b(0));
}

bool ray_line_intersection(const Ray &ray, const Line &line, Out<Vec2> intersection) {
  Eigen::Matrix2d A;
  A.col(0) = ray.direction;
  A.col(1) = -line.direction;

  const Vec2                                        b  = line.point - ray.origin;
  const Eigen::ColPivHouseholderQR<Eigen::Matrix2d> qr = A.colPivHouseholderQr();
  // This takes FOREVER to compile (I guess Eigen inlines the whole operation?)

  // Lines are parallel
  if (qr.absDeterminant() < 1e-3) {
    return false;
  }

  const Vec2 soln = qr.solve(b);
  *intersection   = soln(0) * ray.direction + ray.origin;

  // Ray pointing the wrong direction
  if (soln(0) < 0.0) {
    return false;
  }

  return true;
}

bool ray_line_segment_intersection(const Ray &ray, const LineSegment &segment, Out<Vec2> intersection) {
  const Line temp_line(segment.start, segment.end - segment.start);

  const bool valid_intersection = ray_line_intersection(ray, temp_line, intersection);

  if (valid_intersection) {
    const double half_space_start = (*intersection - segment.start).dot(temp_line.direction);
    const double half_space_end   = (*intersection - segment.end).dot(temp_line.direction);

    if ((half_space_start > 0.0) && (half_space_end < 0.0)) {
      return true;
    }
  }
  return false;
}
}