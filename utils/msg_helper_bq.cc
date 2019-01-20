//%bin(message_helper_bq_main)
//%deps(balsa_queue)
//%deps(message)

#include "infrastructure/balsa_queue/bq_main_macro.hh"
#include "infrastructure/comms/mqtt_comms_factory.hh"

#include "msg_helper_bq.hh"

namespace jet {

MessageHelperBQ::MessageHelperBQ() {
  set_comms_factory(std::make_unique<MqttCommsFactory>());
}

void MessageHelperBQ::init(int argc, char *argv[]) {
  channel_ = "test/topic";
  subscriber_ = make_subscriber(channel_);
  rate_helper_ = std::make_unique<utils::RateHelper>(10);
}

void MessageHelperBQ::loop() {
  std::string message_content;
  if (subscriber_->read_raw(message_content, 0)) {
    Message message;
    message.deserialize(message_content);

    const Timestamp timestamp_ns = message.header.timestamp_ns;
    rate_helper_->update(timestamp_ns);
  }
}

void MessageHelperBQ::shutdown() {
}

}  // namspace jet

BALSA_QUEUE_MAIN_FUNCTION(jet::MessageHelperBQ)
