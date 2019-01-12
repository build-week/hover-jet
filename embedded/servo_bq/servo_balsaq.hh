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
  std::vector<std::pair<ServoDriver, SubscriberPtr>> servo_subscriber_pairs;
  float current_target_percentage;

};

}  // namespace jet
