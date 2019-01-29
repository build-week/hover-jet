#pragma once

#include "viewer/primitives/simple_geometry.hh"

#include "geometry/shapes/circle.hh"

namespace geometry {
namespace visualization {

void put_circle(viewer::SimpleGeometry& geo, const geometry::shapes::Circle& circle);

}  // namespace visualization
}  // namespace geometry