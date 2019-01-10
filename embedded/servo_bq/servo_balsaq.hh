#pragma once

#include <infrastructure/balsa_queue/balsa_queue.hh>
#include "embedded/servo_bq/servo_balsaq.hh"
#include "embedded/servo_bq/set_servo_message.hh"
#include "embedded/servo_driver/servo_driver.hh"
#include "infrastructure/comms/mqtt_comms_factory.hh"

namespace jet {

class ServoBq : public BalsaQ {
 public:
  ServoBq();
  void init(int argc, char *argv[]);
  void loop();
  void shutdown();

 private:
  SubscriberPtr subscriber_;
  std::unique_ptr<ServoDriver> servo;
  int servo_index;
};

}  // namespace jet
