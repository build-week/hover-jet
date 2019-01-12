#pragma once

#include "infrastructure/balsa_queue/balsa_queue.hh"
#include "embedded/servo_bq/set_servo_message.hh"
#include "embedded/servo_driver/servo_driver.hh"
#include "infrastructure/comms/mqtt_comms_factory.hh"

namespace jet {

class ServoBq : public BalsaQ {
 public:
  ServoBq() = default;
  void init(int argc, char *argv[]);
  void loop();
  void shutdown();

 private:
  SubscriberPtr subscriber_;
  std::unique_ptr<ServoDriver> servo;
  float current_target_percentage;

};

}  // namespace jet
