#include "geometry/ray_ray_approx_intersect.hh"

namespace geometry {
IntersectionParameters ray_ray_approx_intersect(const Ray& a, const Ray& b) {
  using Vec3 = Eigen::Vector3d;
  // d->a
  // e->b

  IntersectionParameters result;
  const Vec3             c = b.origin - a.origin;

  const double p = a.direction.dot(b.direction);
  const double q = a.direction.dot(c);
  const double r = b.direction.dot(c);
  const double s = a.direction.dot(a.direction);
  const double t = b.direction.dot(b.direction);

  const double p_sqr         = p * p;
  const double st_min_p2     = (s * t) - p_sqr;
  const double inv_st_min_p2 = 1.0 / st_min_p2;

  constexpr double EPS = 1e-5;
  if (std::abs(s) <= EPS || std::abs(t) < EPS || std::abs(p_sqr - (s * t)) < EPS) {
    result.valid = false;
    return result;
  }

  // const double d2 = (-(p * r) * inv_st_min_p2) + (q / (s - (p_sqr / t)));
  // const double e2 = ((p * q) * inv_st_min_p2) - (r / (t - (p_sqr / s)));
  const double d = ((-p * r) + (q * t)) * inv_st_min_p2;
  const double e = ((p * q) - (r * s)) * inv_st_min_p2;

  result.valid   = true;
  result.along_a = d;
  result.along_b = e;
  return result;
}
}