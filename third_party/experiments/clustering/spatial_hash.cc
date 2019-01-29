#include "clustering/spatial_hash.hh"

#include "geometry/spatial/bounding_box.hh"

#include <limits>
#include <unordered_set>

namespace clustering {
namespace {
using Vec2 = Eigen::Vector2d;
using Vec2Int = Eigen::Matrix<HashInt, 2, 1>;
}

std::vector<HashInt> spatial_hash(const std::vector<Vec2> &points, double scale) {
  geometry::spatial::BoundingBox<2> bbox;
  for (const auto &pt : points) {
    bbox.expand(pt);
  }

  //
  // Force into the nonnegative orthant
  //

  std::vector<HashInt> identities(points.size());

  const Vec2 range = bbox.upper() - bbox.lower();
  const Vec2 lower = bbox.lower();

  for (size_t k = 0; k < points.size(); ++k) {
    const Vec2 normalized = (points[k] - lower).cwiseQuotient(range) * scale;
    const Vec2Int normalized_int = normalized.cast<HashInt>();
    const HashInt hash_value = (0xFFFF0000 & (normalized_int(0) << 16)) | (0x0000FFFF & normalized_int(1));
    identities[k] = hash_value;
  }
  return identities;
}
}
