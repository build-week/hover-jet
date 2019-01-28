#pragma once

#include <infrastructure/balsa_queue/balsa_queue.hh>
#include "embedded/servo_bq/servo_balsaq.hh"
#include "embedded/servo_bq/set_servo_message.hh"
#include "embedded/servo_driver/servo_driver.hh"
#include "infrastructure/comms/mqtt_comms_factory.hh"

namespace jet {
class SingleServoCommandBq : public BalsaQ {
 public:
  SingleServoCommandBq();
  void init(int argc, char *argv[]);
  void loop();
  void shutdown();

 private:
  PublisherPtr publisher_;
  int servo_index;
  float target_angle;
};

}  // namespace jet
