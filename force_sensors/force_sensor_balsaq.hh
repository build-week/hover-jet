
#pragma once

#include <infrastructure/balsa_queue/balsa_queue.hh>
#include "force_sensors/force_sensor_message.hh"
#include "infrastructure/comms/mqtt_comms_factory.hh"

namespace jet {

class ForceSensorBq : public BalsaQ {
 public:
  ForceSensorBq();
  void init(int argc, char *argv[]);
  void loop();
  void shutdown();

 private:
  PublisherPtr publisher_;
};

}  // namespace jet
