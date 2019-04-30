//%bin(gonogo_report_bq_main)

#include "infrastructure/gonogo/gonogo_report_bq.hh"
#include "infrastructure/balsa_queue/bq_main_macro.hh"

#include "infrastructure/comms/mqtt_comms_factory.hh"

#include <iostream>

namespace jet {

namespace {

// Print the contents of an go message
// Tried to make the "GO" and "NOGO" printouts appear visually distinct.
//
// Intentionally not overloading ostream operator
void print_status(const GoNoGoMessage& message) {
  if (message.ready) {
    std::cout << message.bq_name << "GO! (" << message.status_message << ")" << std::endl;
  } else {
    std::cout << message.bq_name << "--- NO go ---(" << message.status_message << ")" << std::endl;
  }
}
}  // namespace

void GoNoGoReportBQ::init(const Config& config) {
  gonogo_sub_ = make_subscriber("GoNoGo");
}

void GoNoGoReportBQ::loop() {
  GoNoGoMessage message;
  while (gonogo_sub_->read(message, 1)) {
    if (!ready_from_bq_name_.count(message.bq_name)) {
      print_status(message);
    } else if (ready_from_bq_name_.at(message.bq_name) != message.ready) {
      print_status(message);
    }

    ready_from_bq_name_[message.bq_name] = message.ready;
  }
}

void GoNoGoReportBQ::shutdown() {
  std::cout << "Shutting down!" << std::endl;
}

}  // namespace jet

BALSA_QUEUE_MAIN_FUNCTION(jet::GoNoGoReportBQ)
