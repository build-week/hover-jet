//%bin(gonogo_bq_main)
//%deps(balsa_queue)
//%deps(message)

#include "infrastructure/gonogo/gonogo_bq.hh"
#include "infrastructure/balsa_queue/bq_main_macro.hh"

#include "infrastructure/comms/mqtt_comms_factory.hh"

#include <iostream>

namespace jet {

void GoNoGoBQ::init(const Config& config) {
  loop_delay_microseconds = 10000;
  gonogo_subscriber_ = make_subscriber("GoNoGo");
  gonogo_state_publisher_ = make_publisher("GoNoGo_output");
}

void GoNoGoBQ::loop() {
  GoNoGoMessage message;
  while (gonogo_subscriber_->read(message, 1)) {
    go_no_go_states_[message.bq_name] = message;
  }

  bool nogo_found = false;
  for (auto& state : go_no_go_states_) {
    if (!state.second.ready) {
      nogo_found = true;
      gonogo_state_publisher_->publish(state.second);
      std::cerr << "NOGO because: " << state.second.bq_name << ":" << state.second.status_message << std::endl;
    }
  }

  if (!nogo_found) {
    GoNoGoMessage go_nogo_message;
    go_nogo_message.ready = true;
    gonogo_state_publisher_->publish(go_nogo_message);
  }
}

void GoNoGoBQ::shutdown() {
  std::cout << "Shutting down!" << std::endl;
}

}  // namespace jet

BALSA_QUEUE_MAIN_FUNCTION(jet::GoNoGoBQ)
