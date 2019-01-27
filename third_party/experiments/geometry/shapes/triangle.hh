#pragma once

#include "util/optional.hh"

#include "eigen.hh"

namespace geometry {
namespace shapes {

using Vec3 = Eigen::Vector3d;

//
// [1] https://iquilezles.org/www/articles/distfunctions/distfunctions.htm
double signed_distance_triangle(const Vec3& point,
                                const Vec3& a,
                                const Vec3& b,
                                const Vec3& c);

// Can you guess what this function does?
Vec3 find_closest_point_on_triangle(const Vec3& point,
                                    const Vec3& a,
                                    const Vec3& b,
                                    const Vec3& c);
}  // namespace shapes
}  // namespace geometry