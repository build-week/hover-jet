//%bin(timesync_client_bq_main)
//%deps(balsa_queue)
//%deps(message)

#include "infrastructure/time/timesync_bq/timesync_client_bq.hh"
#include "infrastructure/balsa_queue/bq_main_macro.hh"

#include "infrastructure/comms/mqtt_comms_factory.hh"
#include "infrastructure/comms/schemas/message.hh"
#include "infrastructure/time/timesync_bq/client_timesync_message.hh"

#include "infrastructure/time/time_utils.hh"

#include <iostream>

namespace jet {


void TimesyncClientBq::init(int argc, char *argv[]) {
  publisher_ = make_publisher("timesync_client");
  subscriber_ = make_subscriber("timesync_master");
}

void TimesyncClientBq::loop() {
  Message master_message;
  if (subscriber_->read(master_message, 1)) {
    ClientTimesyncMessage client_message;
    client_message.master_timestamp = master_message.header.timestamp_ns;
    client_message.master_seq_number = master_message.header.sequence_number;
    // TODO capture potential error here - gethostname will return 0 if successful, SOCKET_ERROR otherwise
    gethostname(client_message.hostname, HOST_NAME_MAX);
    publisher_->publish(client_message);
    std::cout << "cl_name:" << client_message.hostname 
    << " m_ts:" << client_message.master_timestamp 
    << " m_sq:" << client_message.master_seq_number 
    << " cl_ts:" << client_message.header.timestamp_ns
    << " cl_sq:" << client_message.header.sequence_number
    << " offset:" << (client_message.header.timestamp_ns - client_message.master_timestamp) << std::endl;
  }
}

void TimesyncClientBq::shutdown() {
  std::cout << "Shutting down!" << std::endl;
}

}  // namespace jet
BALSA_QUEUE_MAIN_FUNCTION(jet::TimesyncClientBq)
