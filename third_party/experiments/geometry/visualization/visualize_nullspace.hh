#pragma once

#include "viewer/primitives/simple_geometry.hh"

#include "geometry/shapes/circle.hh"

namespace geometry {
namespace visualization {

void visualize_nullspace(viewer::SimpleGeometry& geo,
                         const std::function<double(const Eigen::VectorXd&)>& fnc,
                         const Eigen::VectorXd& x0);

}  // namespace visualization
}  // namespace geometry