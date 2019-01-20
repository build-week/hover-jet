//%bin(force_sensor_balsaq_main)
#include "force_sensors/force_sensor_balsaq.hh"
#include "infrastructure/balsa_queue/bq_main_macro.hh"

#include <cassert>
#include <chrono>
#include <iostream>
#include <thread>

//%deps(balsa_queue)
//%deps(message)

namespace jet {

ForceSensorBq::ForceSensorBq() {
  set_comms_factory(std::make_unique<MqttCommsFactory>());
}

void ForceSensorBq::init(int argc, char *argv[]) {
  assert(argc == 2);
  publisher_ = make_publisher("test/topic");
  //load_cell_reader_ = std::make_unique<LoadCellReceiver>(argv[1]);
}

void ForceSensorBq::loop() {
  ForceSensorMessage message;
  message.header.timestamp_ns = get_current_time();

  message.id         = 1;
  message.value      = 0.123;
  message.timestamp  = message.header.timestamp_ns;

  std::cout << message.header.timestamp_ns << std::endl;
  publisher_->publish(message);
  
  std::this_thread::sleep_for(std::chrono::milliseconds(1230));

  /*
  const auto result = load_cell_reader_->receive();
  if (result) {
    message.id = result->id;
    message.value = result->value;
    message.timestamp = timestamp;
    publisher_->publish(message);
  }
  */
}

void ForceSensorBq::shutdown() {
  std::cout << "Force sensor process shutting down." << std::endl;
}

}  // namespace jet

BALSA_QUEUE_MAIN_FUNCTION(jet::ForceSensorBq)
