//%bin(publisher_demo_bq_main)
//%deps(balsa_queue)
//%deps(message)

#include "infrastructure/comms/tests/pub_sub_demo_bqs/publisher_demo_bq.hh"
#include "infrastructure/balsa_queue/bq_main_macro.hh"

#include "infrastructure/comms/mqtt_comms_factory.hh"
#include "infrastructure/comms/schemas/demo_message.hh"

#include <iostream>

namespace jet {

PublisherDemoBq::PublisherDemoBq() {
  set_comms_factory(std::make_unique<MqttCommsFactory>());
}

void PublisherDemoBq::init() {
  publisher_ = make_publisher("demo_channel_name");
}

void PublisherDemoBq::loop() {
  DemoMessage message;
  message.content = "hello";
  publisher_->publish(message);
}

void PublisherDemoBq::shutdown() {
  std::cout << "Shutting down!" << std::endl;
}

}  // namespace jet
BALSA_QUEUE_MAIN_FUNCTION(jet::PublisherDemoBq)
