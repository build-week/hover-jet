//%bin(servo_balsaq_main)
#include "infrastructure/balsa_queue/bq_main_macro.hh"

#include "visualization/thrust_stand_visualization_bq.hh"
#include "visualization/thrust_stand_visualizer.hh"

// REVISIT
#include <iostream>

namespace jet {
namespace visualization {

void ThrustStandVisualizationBq::init(int argc, char* argv[]) {
  servo_sub_ = make_subscriber("servo_command_channel");
  setup_view();
}

void ThrustStandVisualizationBq::loop() {
  // const JetStatus jet_status({.throttle = 1.0});
  const VaneConfiguration vane_cfg = {};
  const QuadraframeConfiguration qframe_cfg = {};

  SetServoMessage servo_message;

  if (servo_sub_->read(servo_message, 1)) {
    void put_quadraframe(viewer::SimpleGeometry & geo,
                         const control::QuadraframeStatus& status,
                         const control::QuadraframeConfiguration& quad_cfg,
                         const control::VaneConfiguration& vane_cfg);
    const QuadraframeStatus qframe_status = create_quadraframe_status(servo_message);
    put_quadraframe(*geo_, qframe_status, qframe_cfg, vane_cfg);
  }
}
void ThrustStandVisualizationBq::shutdown() {
}
}  // namespace visualization
}  // namespace jet

BALSA_QUEUE_MAIN_FUNCTION(jet::visualization::ThrustStandVisualizationBq)
