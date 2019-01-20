
#pragma once

#include "force_sensors/force_sensor_message.hh"
#include "infrastructure/balsa_queue/balsa_queue.hh"
#include "infrastructure/comms/mqtt_comms_factory.hh"

#include "embedded/load_cell/load_cell.hh"

#include <ctime>

namespace jet {

class ForceSensorBq : public BalsaQ {
 public:
  ForceSensorBq();
  void init(int argc, char *argv[]);
  void loop();
  void shutdown();

 private:
  PublisherPtr publisher_;
  std::unique_ptr<LoadCellReceiver> load_cell_reader_;
};

}  // namespace jet
