//%bin(force_sensor_balsaq_main)
#include "force_sensors/force_sensor_balsaq.hh"
#include "infrastructure/balsa_queue/bq_main_macro.hh"

#include <chrono>
#include <cstddef>
#include <iostream>
#include <sstream>

//%deps(balsa_queue)
//%deps(message)

namespace jet {


ForceSensorBq::ForceSensorBq() {
  set_comms_factory(std::make_unique<MqttCommsFactory>());
}

void ForceSensorBq::init(int argc, char *argv[]) {
  assert(argc == 2);
  force_sensor_index = std::atoi(argv[1]);
  publisher_ = make_publisher("force_sensor_channel_" + std::to_string(force_sensor_index));
}

void ForceSensorBq::loop() {
  std::cout << "Camera BQ: trying to get a frame" << std::endl;	
  ForceSensorMessage message;
  message.force_sensor_index = force_sensor_index;
  message.timestamp = get_current_time();
  publisher_->publish(message);
}
void ForceSensorBq::shutdown() {
  std::cout << "force sensor process shutting down." << std::endl;
}

}  // namespace jet

BALSA_QUEUE_MAIN_FUNCTION(jet::ForceSensorBq)
