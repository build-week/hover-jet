//%bin(timesync_client_bq_main)
//%deps(balsa_queue)
//%deps(message)

#include "infrastructure/time/timesync_bq/timesync_client_bq.hh"
#include "infrastructure/balsa_queue/bq_main_macro.hh"

#include "infrastructure/comms/mqtt_comms_factory.hh"
#include "infrastructure/comms/schemas/message.hh"

#include "infrastructure/time/time_utils.hh"

#include <iostream>

namespace jet {


void TimesyncClientBq::init(int argc, char *argv[]) {
  publisher_ = make_publisher("timesync_client");
  subscriber_ = make_subscriber("timesync_master");
  // gethostname will return 0 if successful, SOCKET_ERROR otherwise
  if (gethostname(client_message_.hostname, HOST_NAME_MAX) != 0) {
    gonogo_.nogo("Socket Error getting hostname");
  } else {
    gonogo_.go();
  }
}

void TimesyncClientBq::loop() {
  Message master_message;
  if (subscriber_->read(master_message, 1)) {
    client_message_.master_timestamp = master_message.header.timestamp_ns;
    client_message_.master_seq_number = master_message.header.sequence_number;

    publisher_->publish(client_message_);
    std::cout << "cl_name:" << client_message_.hostname 
    << " m_ts:" << client_message_.master_timestamp 
    << " m_sq:" << client_message_.master_seq_number 
    << " cl_ts:" << client_message_.header.timestamp_ns
    << " cl_sq:" << client_message_.header.sequence_number
    << " offset:" << (client_message_.header.timestamp_ns - client_message_.master_timestamp) << std::endl;
  }
}

void TimesyncClientBq::shutdown() {
  std::cout << "Timesync client shutting down!" << std::endl;
}

}  // namespace jet
BALSA_QUEUE_MAIN_FUNCTION(jet::TimesyncClientBq)
