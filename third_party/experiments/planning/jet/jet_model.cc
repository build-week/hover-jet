#include "planning/jet/jet_model.hh"

#include "geometry/visualization/put_collada.hh"
#include "viewer/primitives/simple_geometry.hh"

namespace planning {
namespace jet {

JetModel::JetModel() : model_(jcc::Environment::asset_path() + "Hover-Jet Vehicle.dae") {
}

void JetModel::insert(viewer::SceneTree& tree) const {
  const SE3 world_from_jet = SE3();

  const auto dummy_geo = std::make_shared<viewer::SimpleGeometry>();
  tree.add_primitive("root", world_from_jet, model_.root(), dummy_geo);

  std::map<std::string, SE3> world_from_node;
  world_from_node[model_.root()] = world_from_jet;

  std::stack<std::string> to_visit;
  to_visit.push(model_.root());

  const auto& adj = model_.adjacency();
  const auto& meshes = model_.meshes();
  const auto& colors = model_.colors();

  while (!to_visit.empty()) {
    const auto key = to_visit.top();
    to_visit.pop();

    if (adj.count(key) == 0) {
      continue;
    }

    const SE3 world_from_parent = world_from_node.at(key);

    for (const auto& edge : adj.at(key)) {
      world_from_node[edge.child_name] =
          world_from_parent * edge.child_from_parent.inverse();
      to_visit.push(edge.child_name);

      const auto geo = std::make_shared<viewer::SimpleGeometry>();
      const auto& mesh = meshes.at(edge.child_name);
      if (colors.count(edge.child_name)) {
        const jcc::Vec4 color = colors.at(edge.child_name);
        geo->add_triangle_mesh({mesh, SE3(), color, true, 3.0, false});
      }

      geo->flip();
      tree.add_primitive(key, edge.child_from_parent.inverse(), edge.child_name, geo);
    }
  }
}

}  // namespace jet
}  // namespace planning
