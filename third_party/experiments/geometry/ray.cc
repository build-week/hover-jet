#include "geometry/ray.hh"

namespace geometry {

Ray operator*(const SE3& destination_from_source, const Ray& ray) {
  Ray new_ray;
  new_ray.origin    = destination_from_source * ray.origin;
  new_ray.direction = destination_from_source.so3() * ray.direction;
  return new_ray;
}
}
