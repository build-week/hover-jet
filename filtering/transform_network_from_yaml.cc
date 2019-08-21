#include "filtering/transform_network_from_yaml.hh"

#include "filtering/yaml_matrix.hh"

#include <set>

namespace jet {
namespace {
void se3_to_yaml(YAML::Node& parent_yaml, const SE3& source_from_destination) {
  set_matrix(parent_yaml, "log_R_source_from_destination", source_from_destination.so3().log());
  set_matrix(parent_yaml, "translation_source_from_destination", source_from_destination.translation());
}
}  // namespace

geometry::TransformNetwork transform_network_from_yaml(const YAML::Node& node) {
  geometry::TransformNetwork tfn;
  for (const auto& subnode : node) {
    const auto source = subnode["source"].as<std::string>();
    const auto destination = subnode["destination"].as<std::string>();

    if (subnode["log_source_from_destination"]) {
      const auto log_source_from_destination = subnode["log_source_from_destination"].as<std::vector<double>>();
      if (log_source_from_destination.size() == 3u) {
        //
        // SO3 - log
        //
        const SE3 source_from_destination =
            SE3(SO3::exp(read_matrix<3, 1>(subnode["log_source_from_destination"])), jcc::Vec3::Zero());
        tfn.add_edge(source, destination, source_from_destination);
      } else if (log_source_from_destination.size() == 6u) {
        //
        // SE3 - log
        //
        const SE3 source_from_destination = SE3::exp(read_matrix<6, 1>(subnode["log_source_from_destination"]));
        tfn.add_edge(source, destination, source_from_destination);
      }
    } else if (subnode["source_from_destination"]) {
      //
      // SE3 - log(SO3) & translation
      //
      const YAML::Node se3_node = subnode["source_from_destination"];
      const SO3 R_source_from_destination = SO3::exp(read_matrix<3, 1>(se3_node["log_R_source_from_destination"]));
      const jcc::Vec3 translation_source_from_destination =
          read_matrix<3, 1>(se3_node["translation_source_from_destination"]);
      const SE3 source_from_destination = SE3(R_source_from_destination, translation_source_from_destination);
      tfn.add_edge(source, destination, source_from_destination);
    }
    // TODO(jpanikulam): Support for storing as {log(rot), translation-vector} pair
  }
  return tfn;
}
void transform_network_to_yaml(YAML::Node& parent_yaml, const geometry::TransformNetwork& tfn) {
  // Only store each edge once
  std::set<std::string> visited;

  YAML::Node tf_yamls;
  parent_yaml["transforms"] = tf_yamls;
  for (const auto& node : tfn.edges_from_node_tag()) {
    YAML::Node node_yaml;

    bool added_any = false;
    for (const auto& edge : node.second) {
      const bool should_add =
          (visited.find(node.first) != visited.end()) || (visited.find(edge.first) != visited.end());
      if (should_add) {
        node_yaml["source"] = node.first;
        node_yaml["destination"] = edge.first;
        YAML::Node se3_yaml;
        node_yaml["source_from_destination"] = se3_yaml;
        se3_to_yaml(se3_yaml, edge.second.source_from_destination);
        added_any = should_add;
      }
      visited.insert(edge.first);
    }
    if(added_any) {
      tf_yamls.push_back(node_yaml);
    }
    visited.insert(node.first);
  }
}

}  // namespace jet
