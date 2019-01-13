#pragma once

#include <vector>

#include "infrastructure/balsa_queue/balsa_queue.hh"
#include "infrastructure/comms/mqtt_comms_factory.hh"

#include "control/servo_interface.hh"

namespace jet {
namespace control {

class ThrustStandControlTestBq : public BalsaQ {
 public:
  const static uint loop_delay_microseconds = 25000;

  ThrustStandControlTestBq() = default;

  // The first argument passed to the this must be a path to a yaml config.
  //
  // The yaml is of the following format:
  //
  // commands:
  //   - [servo_angle_0_rads, servo_angle_1_rads, servo_angle_2_rads, servo_angle_3_rads]
  //   - [servo_angle_0_rads, servo_angle_1_rads, servo_angle_2_rads, servo_angle_3_rads]
  //   - [servo_angle_0_rads, servo_angle_1_rads, servo_angle_2_rads, servo_angle_3_rads]
  //   - [servo_angle_0_rads, servo_angle_1_rads, servo_angle_2_rads, servo_angle_3_rads]
  //
  // Each command is separated from the previous by ~25ms, governed by this bq's loop time
  // Yes, that will make these sequences *huge*
  //
  void init(int argc, char *argv[]) override;
  void loop() override;
  void shutdown() override;

 private:
  PublisherPtr servo_pub_;

  int command_index_ = 0;
  std::vector<QuadraframeStatus> command_sequence_;
};

}  // namespace control
}  // namespace jet
