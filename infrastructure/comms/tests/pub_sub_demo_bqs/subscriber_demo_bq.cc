
//%deps(balsa_queue)
//%deps(message)

#include "infrastructure/comms/tests/pub_sub_demo_bqs/subscriber_demo_bq.hh"
#include "infrastructure/balsa_queue/bq_main_macro.hh"

#include "infrastructure/comms/mqtt_comms_factory.hh"
#include "infrastructure/comms/schemas/demo_message.hh"

#include <iostream>

namespace jet {

void SubscriberDemoBq::init(int argc, char *argv[]) {
  subscriber_ = make_subscriber("demo_channel_name");
}

void SubscriberDemoBq::loop() {
  DemoMessage message;
  if (subscriber_->read(message, 1)) {
    std::cout << "At " << message.header.timestamp_ns
              << " SubscriberDemoBq received message #: "
              << message.header.sequence_number << " with content: " << message.content
              << std::endl;
  }
}

void SubscriberDemoBq::shutdown() {
  std::cout << "Shutting down!" << std::endl;
}

}  // namespace jet

BALSA_QUEUE_MAIN_FUNCTION(jet::SubscriberDemoBq)
