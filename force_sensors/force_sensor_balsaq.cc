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
  publisher_ = make_publisher("force_sensor_output_channel");
}

void ForceSensorBq::loop() {
  std::cout << "Camera BQ: trying to get a frame" << std::endl;	
  ForceSensorMessage message;

  //TODO Ralph: put some data in this vector
  message.values.push_back(1);
  message.values.push_back(2);
  message.values.push_back(3);
  publisher_->publish(message);
}
void ForceSensorBq::shutdown() {
  std::cout << "force sensor process shutting down." << std::endl;
}

}  // namespace jet

BALSA_QUEUE_MAIN_FUNCTION(jet::ForceSensorBq)
