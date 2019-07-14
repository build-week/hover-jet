//%bin(comms_latency_test_subscriber_bq_main)
//%deps(balsa_queue)
//%deps(message)

#include "infrastructure/comms/tests/comms_latency_test/latency_test_subscriber_bq.hh"
#include "infrastructure/balsa_queue/bq_main_macro.hh"

#include "infrastructure/comms/mqtt_comms_factory.hh"

#include <iostream>

namespace jet {

void CommsLatencyTestSubscriberBq::init(const Config& config) {
  // subscriber_ = make_subscriber("comms_latency_test_channel_name");
  shared_struct_ = std::make_unique<SharedStructSubscriber>("comms_latency_test_channel_name");
}

void CommsLatencyTestSubscriberBq::loop() {
  // std::cout << "Time since last loop: " << get_current_time() - previous_loop_time_ << std::endl;
  // previous_loop_time_ = get_current_time();

  // LatencyProbeMessage message;
  // if (subscriber_->read(message, 1)) {
  //   auto time_diff_ns = get_current_time()  - message.header.timestamp_ns;
  //   ring_buffer_.insert_into_ring_buffer(time_diff_ns);
  // 	total_time_between_messages_ns_ += time_diff_ns;
  // 	total_message_count_++;
  // 	if (total_message_count_ > 0 && total_message_count_ % 10 == 0) {
  // 		std::cerr << "Avg time between publish and read: " << total_time_between_messages_ns_ / total_message_count_ / 1000000.0 << "ms" << std::endl;
  // 		std::cerr << "Avg time in last 10 samples: " << ring_buffer_.get_avg_from_ring_buffer() << "ms" << std::endl;
  // 	}
  // }


  LatencyProbeMessage message_from_shared_struct;
  shared_struct_->read(message_from_shared_struct);
  if (recent_sequence_number_ != message_from_shared_struct.header.sequence_number && message_from_shared_struct.header.timestamp_ns != 0) {
    recent_sequence_number_ = message_from_shared_struct.header.sequence_number;
    auto time_diff_ns = get_current_time()  - message_from_shared_struct.header.timestamp_ns;
    ring_buffer_.insert_into_ring_buffer(time_diff_ns);
   total_time_between_messages_ns_ += time_diff_ns;
   total_message_count_++;
   if (total_message_count_ > 0 && total_message_count_ % 10 == 0) {
     std::cerr << "Avg time between publish and read: " << total_time_between_messages_ns_ / total_message_count_ / 1000000.0 << "ms" << std::endl;
     std::cerr << "Avg time in last 10 samples: " << ring_buffer_.get_avg_from_ring_buffer() << "ms" << std::endl;
   }
  }

   


  // LatencyProbeMessage message_from_shared_struct;
  // shared_struct_->read(message_from_shared_struct);
  // std::cout << "Shared memory object header timestamp: " << message_from_shared_struct.header.timestamp_ns << std::endl;
}

void CommsLatencyTestSubscriberBq::shutdown() {
  std::cout << "Shutting down!" << std::endl;
}

}  // namespace jet

BALSA_QUEUE_MAIN_FUNCTION(jet::CommsLatencyTestSubscriberBq)
