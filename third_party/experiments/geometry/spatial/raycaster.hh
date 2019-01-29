#pragma once

#include "geometry/spatial/volume.hh"
#include "geometry/ray.hh"

#include "sophus.hh"

#include <memory>
#include <vector>

namespace geometry {
namespace spatial {

struct RayCasterConfig {
  std::vector<Ray> rays;
  double max_range = 50.0;
};

RayCasterConfig build_raycaster_config();

class RayCaster {
public:
  RayCaster(const RayCasterConfig &config)
      : config_(config){};
  const RayCasterConfig &config() & {
    return config_;
  }

  void add_volume(std::shared_ptr<Volume> volume);
  std::vector<double> cast_rays(const SE3 &world_from_caster);

private:
  RayCasterConfig config_;
  std::vector<std::shared_ptr<Volume>> volumes_;
};
}
}
