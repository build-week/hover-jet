//%deps(balsa_queue)
//%deps(message)

#include "infrastructure/gonogo/gonogo_bq.hh"
#include "infrastructure/balsa_queue/bq_main_macro.hh"

#include "infrastructure/comms/mqtt_comms_factory.hh"

#include <iostream>

namespace jet {

void GoNoGoBQ::init(int argc, char *argv[]) {
  loop_delay_microseconds = 10000;
  gonogo_subscriber_ = make_subscriber("GoNoGo");
  gonogo_state_publisher_ = make_publisher("GoNoGo_output");
}

void GoNoGoBQ::loop() {
  GoNoGoMessage message;
  while (gonogo_subscriber_->read(message, 1)) {
    go_no_go_states_[message.bq_name] = message;
  }
  for (const auto& state : go_no_go_states_)
  {
    if (!state.second.ready) {
      GoNoGoMessage go_nogo_message;
      go_nogo_message.ready = false;
      gonogo_state_publisher_->publish(go_nogo_message);
      std::cerr << "NOGO because: " << go_nogo_message.bq_name << ":" << go_nogo_message.status_message << std::endl;
    }
  }
}

void GoNoGoBQ::shutdown() {
  std::cout << "Shutting down!" << std::endl;
}

}  // namespace jet

BALSA_QUEUE_MAIN_FUNCTION(jet::GoNoGoBQ)
