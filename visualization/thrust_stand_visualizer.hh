#pragma once

// %deps(simple_geometry)
// %deps(window_3d)
#include "third_party/experiments/viewer/primitives/simple_geometry.hh"
#include "third_party/experiments/viewer/window_3d.hh"

// #include "visualization/test_stand_mocks.hh"
#include "control/jet_vane_model.hh"
#include "control/quadraframe_model.hh"
#include "control/vanes_generated.hh"

namespace jet {
namespace visualization {

void setup_view(const std::string& viewer_name);

void put_quadraframe(viewer::SimpleGeometry& geo,
                     const control::QuadraframeStatus& status,
                     const control::QuadraframeConfiguration& quad_cfg,
                     const control::VaneConfiguration& vane_cfg);
}  // namespace visualization
}  // namespace jet