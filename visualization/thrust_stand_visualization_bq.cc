//%bin(thrust_stand_visualization_bq_main)
#include "infrastructure/balsa_queue/bq_main_macro.hh"

#include "visualization/thrust_stand_visualization_bq.hh"
#include "visualization/thrust_stand_visualizer.hh"

// REVISIT
#include <iostream>

namespace jet {
namespace visualization {

void ThrustStandVisualizerBq::init(int argc, char* argv[]) {
  servo_sub_ = make_subscriber("servo_command_channel");
  setup_view();
}

void ThrustStandVisualizerBq::loop() {
  const control::VaneConfiguration vane_cfg = {};
  const control::QuadraframeConfiguration qframe_cfg = {};

  SetServoMessage servo_message;
  if (servo_sub_->read(servo_message, 1)) {
    std::cout << "Got message" << std::endl;
    const control::QuadraframeStatus qframe_status =
        control::create_quadraframe_status(servo_message);
    put_quadraframe(*geo_, qframe_status, qframe_cfg, vane_cfg);
  }
}
void ThrustStandVisualizerBq::shutdown() {
}
}  // namespace visualization
}  // namespace jet

BALSA_QUEUE_MAIN_FUNCTION(jet::visualization::ThrustStandVisualizerBq)
