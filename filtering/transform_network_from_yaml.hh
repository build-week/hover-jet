#pragma once

// %deps(transform_network)
#include "third_party/experiments/geometry/kinematics/transform_network.hh"

// %deps(yaml-cpp)
#include <yaml-cpp/yaml.h>

namespace jet {

// Create a transform network from a yaml
geometry::TransformNetwork transform_network_from_yaml(const YAML::Node& node);
void transform_network_to_yaml(YAML::Node& parent_yaml, const geometry::TransformNetwork& tfn);
}  // namespace jet
