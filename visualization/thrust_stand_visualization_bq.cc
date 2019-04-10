//%bin(thrust_stand_visualization_bq_main)
#include "infrastructure/balsa_queue/bq_main_macro.hh"

#include "force_sensors/force_sensor_message.hh"
#include "visualization/thrust_stand_visualization_bq.hh"
#include "visualization/thrust_stand_visualizer.hh"

namespace jet {
namespace visualization {

void ThrustStandVisualizerBq::init(const Config& config) {
  servo_sub_ = make_subscriber("servo_command_channel");
  load_cell_sub_ = make_subscriber("force_sensor_output_channel");

  const std::string viewer_name = "Mr. Thrust Stand Visualizer, Visualizes";
  view_ = viewer::get_window3d(viewer_name);
  servo_geo_ = view_->add_primitive<viewer::SimpleGeometry>();
  force_geo_ = view_->add_primitive<viewer::SimpleGeometry>();

  setup_view(viewer_name);

  // Draw something before we have get a message
  put_quadraframe(*servo_geo_, {}, {}, {});
  servo_geo_->flip();
}

void ThrustStandVisualizerBq::loop() {
  const control::VaneConfiguration vane_cfg = {};
  const control::QuadraframeConfiguration qframe_cfg = {};

  ForceSensorMessage load_cell_message;
  while (load_cell_sub_->read(load_cell_message, 1)) {
    thrust_stand_status_.load_cell_value_from_id[load_cell_message.id] =
        load_cell_message.value;
  }

  put_thrust_stand(*force_geo_, thrust_stand_status_);
  force_geo_->flip();

  // Print thrust stand values in a structured way
  if (print_thrust_stand_values_) {
    std::cout << "--\n\n" << std::endl;
    std::cout << "0: " << thrust_stand_status_.load_cell_value_from_id[0] << std::endl;
    std::cout << "1: " << thrust_stand_status_.load_cell_value_from_id[1] << std::endl;
    std::cout << "2: " << thrust_stand_status_.load_cell_value_from_id[2] << std::endl;
    std::cout << "3: " << thrust_stand_status_.load_cell_value_from_id[3] << std::endl;
    std::cout << "4: " << thrust_stand_status_.load_cell_value_from_id[4] << std::endl;
    std::cout << "5: " << thrust_stand_status_.load_cell_value_from_id[5] << std::endl;
  }

  SetServoMessage servo_message;
  bool got_servo_msg = false;
  while (servo_sub_->read(servo_message, 1)) {
    got_servo_msg = true;
  }
  if (got_servo_msg) {
    const control::QuadraframeStatus qframe_status =
        control::create_quadraframe_status(servo_message);
    put_quadraframe(*servo_geo_, qframe_status, qframe_cfg, vane_cfg);
    servo_geo_->flip();
  }
}
void ThrustStandVisualizerBq::shutdown() {
}
}  // namespace visualization
}  // namespace jet

BALSA_QUEUE_MAIN_FUNCTION(jet::visualization::ThrustStandVisualizerBq)
