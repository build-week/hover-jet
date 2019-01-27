#include "geometry/shapes/triangle.hh"

#include <algorithm>

#include "util/clamp.hh"
#include "util/sign.hh"

namespace geometry {
namespace shapes {

using Vec3 = Eigen::Vector3d;

namespace {
inline double dot2(const Vec3& v) {
  return v.dot(v);
}
}  // namespace

//
// [1] https://iquilezles.org/www/articles/distfunctions/distfunctions.htm
double signed_distance_triangle(const Vec3& point,
                                const Vec3& a,
                                const Vec3& b,
                                const Vec3& c) {
  const Vec3 ba = b - a;
  const Vec3 pa = point - a;
  const Vec3 cb = c - b;
  const Vec3 pb = point - b;
  const Vec3 ac = a - c;
  const Vec3 pc = point - c;
  const Vec3 normal = ba.cross(ac);

  // This can be heavily optimized
  const double inside_triangle = jcc::sign((ba.cross(normal)).dot(pa)) +
                                 jcc::sign((cb.cross(normal)).dot(pb)) +
                                 jcc::sign((ac.cross(normal)).dot(pc));
  double distance = -1.0;
  if (inside_triangle < 2.0) {
    distance = std::min({dot2(ba * jcc::clamp(ba.dot(pa) / dot2(ba), 0.0, 1.0) - pa),
                         dot2(cb * jcc::clamp(cb.dot(pb) / dot2(cb), 0.0, 1.0) - pb),
                         dot2(ac * jcc::clamp(ac.dot(pc) / dot2(ac), 0.0, 1.0) - pc)});
  } else {
    distance = (normal.dot(pa) * normal.dot(pa)) / dot2(normal);
  }

  return std::sqrt(distance);
}

Vec3 find_closest_point_on_triangle(const Vec3& point,
                                    const Vec3& a,
                                    const Vec3& b,
                                    const Vec3& c) {
  /*  const Vec3 u = b - a;
    const Vec3 v = c - a;
    const Vec3 n = u.cross(v);
    const Vec3 w = point - a;

    const double dot2n = dot2(n);
    const double gamma = (u.cross(w).dot(n)) / dot2n;
    const double beta = (w.cross(v).dot(n)) / dot2n;
    const double alpha = 1.0 - gamma - beta;

    const auto is_in_range = [](double query, double min, double max) {
      return (query <= max) && (query >= min);
    };

    const bool in_triangle = (is_in_range(alpha, 0.0, 1.0) && is_in_range(beta, 0.0, 1.0)
    && is_in_range(gamma, 0.0, 1.0));

    if (!in_triangle) {
    } else {
      return (alpha * a) + (beta * b) + (gamma * c);
    }
    return {projection};
  */

  const Vec3 ba = b - a;
  const Vec3 pa = point - a;
  const Vec3 cb = c - b;
  const Vec3 pb = point - b;
  const Vec3 ac = a - c;
  const Vec3 pc = point - c;
  const Vec3 normal = ba.cross(ac);

  // This can be heavily optimized
  const double inside_triangle = jcc::sign((ba.cross(normal)).dot(pa)) +
                                 jcc::sign((cb.cross(normal)).dot(pb)) +
                                 jcc::sign((ac.cross(normal)).dot(pc));
  if (inside_triangle < 2.0) {
    const Vec3 pt_ba = ba * jcc::clamp(ba.dot(pa) / dot2(ba), 0.0, 1.0) + a;
    const Vec3 pt_cb = cb * jcc::clamp(cb.dot(pb) / dot2(cb), 0.0, 1.0) + b;
    const Vec3 pt_ac = ac * jcc::clamp(ac.dot(pc) / dot2(ac), 0.0, 1.0) + c;

    const std::initializer_list<std::reference_wrapper<const Vec3>> vv = {pt_ba, pt_cb,
                                                                          pt_ac};

    const Vec3& d = *std::min_element(
        vv.begin(), vv.end(), [&point](const Vec3& pt_a, const Vec3& pt_b) {
          return (point - pt_a).squaredNorm() < (point - pt_b).squaredNorm();
        });
    return d;

  } else {
    return point - (-normal * (-normal.dot(pa) / dot2(-normal)));
  }
}
}  // namespace shapes
}  // namespace geometry