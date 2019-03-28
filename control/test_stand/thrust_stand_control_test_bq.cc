// %bin(thrust_stand_control_test_balsaq_main)
#include "control/test_stand/thrust_stand_control_test_bq.hh"

#include "infrastructure/balsa_queue/bq_main_macro.hh"

// %deps(yaml-cpp)
#include <cassert>

namespace jet {
namespace control {

namespace {
// Generate a servo command from the pre-ordained yaml format
QuadraframeStatus servo_commands_from_angle_list(const YAML::Node& yml_node) {
  QuadraframeStatus status;
  status.servo_0_angle_rad = yml_node[0].as<double>();
  status.servo_1_angle_rad = yml_node[1].as<double>();
  status.servo_2_angle_rad = yml_node[2].as<double>();
  status.servo_3_angle_rad = yml_node[3].as<double>();
  return status;
}
}  // namespace

void ThrustStandControlTestBq::init(const Config& config) {
  //
  // Read configuration
  //

  for (const auto& command : config["commands"]) {
    const auto qframe_status = servo_commands_from_angle_list(command);
    command_sequence_.push_back(qframe_status);
  }

  assert(!command_sequence_.empty());

  //
  // Set up IPC
  //

  servo_pub_ = make_publisher("servo_command_channel");
}

void ThrustStandControlTestBq::loop() {
  //
  // Manage location in the command sequence
  //

  command_index_ += 1;
  if (command_index_ % 500 == 0) {
    std::cout << "Issuing command: " << command_index_ << std::endl;
  }

  const int max_command_index = static_cast<int>(command_sequence_.size()) - 1;
  if (command_index_ > max_command_index) {
    std::cout << "Restarting command sequence" << std::endl;
    command_index_ = 0;
  }

  //
  // Issue commands
  //

  const auto target_qframe_status = command_sequence_.at(command_index_);
  SetServoMessage servo_message = create_servo_command(target_qframe_status);
  // TODO: Is this method truly non-const?
  servo_pub_->publish(servo_message);
}

void ThrustStandControlTestBq::shutdown() {
}

}  // namespace control
}  // namespace jet

BALSA_QUEUE_MAIN_FUNCTION(jet::control::ThrustStandControlTestBq)
