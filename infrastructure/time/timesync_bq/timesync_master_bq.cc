//%bin(timesync_master_bq_main)
//%deps(balsa_queue)
//%deps(message)

#include "infrastructure/time/timesync_bq/timesync_master_bq.hh"
#include "infrastructure/balsa_queue/bq_main_macro.hh"

#include "infrastructure/comms/mqtt_comms_factory.hh"
#include "infrastructure/comms/schemas/message.hh"

#include <iostream>

namespace jet {


void TimesyncMasterBq::init(int argc, char *argv[]) {
  publisher_ = make_publisher("timesync_master");
}

void TimesyncMasterBq::loop() {
  Message message;
  // the message doesn't need content since the header will contain the timestamp
  publisher_->publish(message);
}

void TimesyncMasterBq::shutdown() {
  std::cout << "Shutting down!" << std::endl;
}

}  // namespace jet
BALSA_QUEUE_MAIN_FUNCTION(jet::TimesyncMasterBq)
