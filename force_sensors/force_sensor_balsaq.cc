//%bin(force_sensor_balsaq_main)
#include "force_sensors/force_sensor_balsaq.hh"
#include "infrastructure/balsa_queue/bq_main_macro.hh"

#include <cassert>
#include <iostream>

//%deps(balsa_queue)
//%deps(message)

namespace jet {

ForceSensorBq::ForceSensorBq() {
  set_comms_factory(std::make_unique<MqttCommsFactory>());
}

void ForceSensorBq::init(const Config& config) {
  publisher_ = make_publisher("force_sensor_output_channel");
  load_cell_reader_ = std::make_unique<LoadCellReceiver>(config["reader"].as<std::string>());
}

void ForceSensorBq::loop() {
  ForceSensorMessage message;

  const auto timestamp = get_current_time();
  const auto result = load_cell_reader_->receive();
  if (result) {
    message.id = result->id;
    message.value = result->value;
    message.timestamp = timestamp;
    publisher_->publish(message);
  }
}

void ForceSensorBq::shutdown() {
  std::cout << "Force sensor process shutting down." << std::endl;
}

}  // namespace jet

BALSA_QUEUE_MAIN_FUNCTION(jet::ForceSensorBq)
