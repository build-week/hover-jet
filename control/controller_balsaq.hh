#pragma once

#include "infrastructure/balsa_queue/balsa_queue.hh"
#include "infrastructure/comms/mqtt_comms_factory.hh"

#include "control/quadraframe_model.hh"

namespace jet {
namespace control {

class ControllerBq : public BalsaQ {
 public:
  ControllerBq();
  void init(const Config& config);
  void loop();
  void shutdown();

  // Every 10 milliseconds
  const static uint loop_delay_microseconds = 10000;

 private:
  SubscriberPtr roll_sub_;
  SubscriberPtr pitch_sub_;
  SubscriberPtr yaw_sub_;
  double joy_roll_ = 0.0;
  double joy_pitch_ = 0.0;
  double joy_yaw_ = 0.0;

  SubscriberPtr pose_sub_;

  SubscriberPtr turbine_state_sub_;

  bool got_turbine_status_;
  JetStatus jet_status_;

  PublisherPtr servo_pub_;
};

}  // namespace control
}  // namespace jet
