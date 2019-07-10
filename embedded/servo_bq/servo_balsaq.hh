#pragma once

#include "embedded/servo_driver/servo_driver.hh"
#include "infrastructure/balsa_queue/balsa_queue.hh"
#include "infrastructure/comms/mqtt_comms_factory.hh"

namespace jet {

class ServoBq : public BalsaQ {
 public:
  ServoBq() = default;
  void init(const Config& config);
  void loop();
  void shutdown();

 private:
  std::vector<ServoDriver> servos_;
  SubscriberPtr subscriber_;
  PublisherPtr publisher_;
  Timestamp last_msg_recvd_timestamp_;
};

}  // namespace jet
