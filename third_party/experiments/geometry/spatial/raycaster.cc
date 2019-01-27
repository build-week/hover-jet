#include "geometry/spatial/raycaster.hh"

#include <limits>

namespace geometry {
namespace spatial {
using Vec3 = Eigen::Vector3d;

RayCasterConfig build_raycaster_config() {
  RayCasterConfig cfg;
  constexpr double HALF_PI = M_PI * 0.5;
  constexpr double QTR_PI = HALF_PI * 0.5;
  constexpr double SPACING = 0.1;

  cfg.rays.push_back({Vec3::Zero(), Vec3(1.0, 0.0, 0.0).normalized()});

  // for (double az = -HALF_PI; az <= HALF_PI; az += SPACING) {
  for (double az = -QTR_PI; az <= QTR_PI; az += SPACING) {
    // break;
    for (double el = 0.0; el <= 0.0; el += SPACING) {
      const double z = std::sin(el);
      const double r = std::cos(el);
      const double y = std::sin(az) * r;
      const double x = std::cos(az) * r;

      cfg.rays.push_back({Vec3::Zero(), Vec3(x, y, z).normalized()});
    }
  }
  return cfg;
};

void RayCaster::add_volume(std::shared_ptr<Volume> volume) {
  volumes_.push_back(volume);
}

std::vector<double> RayCaster::cast_rays(const SE3 &world_from_caster) {
  std::vector<double> ranges(config_.rays.size(), std::numeric_limits<double>::max());
  for (size_t i_ray = 0; i_ray < ranges.size(); ++i_ray) {
    const Ray ray = world_from_caster * config_.rays[i_ray];
    for (size_t i_vol = 0; i_vol < volumes_.size(); ++i_vol) {
      const auto intersection = volumes_[i_vol]->intersect(ray);
      if (intersection.intersected && intersection.distance < ranges[i_ray]) {
        ranges[i_ray] = intersection.distance;
      }
    }
  }
  return ranges;
}
}
}
