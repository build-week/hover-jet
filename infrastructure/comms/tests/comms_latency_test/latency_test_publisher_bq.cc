//%bin(comms_latency_test_publisher_bq_main)
//%deps(balsa_queue)
//%deps(message)

#include "infrastructure/comms/tests/comms_latency_test/latency_test_publisher_bq.hh"
#include "infrastructure/balsa_queue/bq_main_macro.hh"

// #include "infrastructure/comms/mqtt_synchronous_publisher.hh"
#include "infrastructure/comms/mqtt_comms_factory.hh"
#include "infrastructure/comms/tests/comms_latency_test/latency_probe_message.hh"

#include <iostream>
#include <iomanip>

namespace jet {


void CommsLatencyTestPublisherBq::init(const Config& config) {
  assert(config["message_size_bytes"]);
  assert(config["loop_delay_us"]);

  loop_delay_microseconds = config["loop_delay_us"].as<uint32_t>();
  // publisher_ = std::make_unique<jet::MqttSynchronousPublisher>("comms_latency_test_channel_name");
  shared_struct_ = std::make_unique<jet::SharedStructPublisher>("comms_latency_test_channel_name");
  // publisher_ = make_publisher("comms_latency_test_channel_name");

  std::stringstream payload_stream;
  uint32_t payload_size_bytes = config["message_size_bytes"].as<uint32_t>();
  for (uint32_t i = 0; i < payload_size_bytes; ++i) {
  	payload_stream << 'a';
  }
  payload_ = payload_stream.str();

  std::cout << "Running" << std::endl;
  float message_per_second = loop_delay_microseconds / 1000000.0;
  std::cout << "mesasges / second: " << message_per_second  << std::endl;
  std::cout << "message size: " << payload_size_bytes << " bytes" << std::endl;
  std::cout << "Estimated bandwidth: " << (1.0 / message_per_second) * payload_size_bytes / 1000.0 << " KBytes per second" << std::endl;

}

void CommsLatencyTestPublisherBq::loop() {
  auto start_time = get_current_time();
  LatencyProbeMessage message;
  uint64_t time_to_create_message = get_current_time() - start_time;
  if (time_to_create_message > 2000000){
    std::cout << "Time to create probe message: " << std::fixed << std::setprecision(1) << time_to_create_message << "ns" << std::endl;
  }

  auto payload_copy_start_time = get_current_time();
  message.content = payload_;
  uint64_t time_to_copy_payload = get_current_time() - payload_copy_start_time;
  if (time_to_copy_payload > 2000000){
    std::cout << "Time to copy payload: " << std::fixed <<  std::setprecision(1) << time_to_copy_payload << "ns" << std::endl;
  }

  auto publisher_start_time = get_current_time();
  // publisher_->publish(message);
  uint64_t time_to_publish = get_current_time() - publisher_start_time;
  if (time_to_publish > 2000000){
    std::cout << "Time to publish: " << std::fixed <<  std::setprecision(1) << time_to_publish << "ns" << std::endl;
  }

  LatencyProbeMessage struct_message;
  struct_message.content = payload_;
  auto write_start_time = get_current_time();
  shared_struct_->publish(struct_message);
  uint64_t time_to_write = get_current_time() - write_start_time;
  // if (time_to_write > 2000000){
  std::cout << "Time to write: " << std::fixed <<  std::setprecision(1) << time_to_write << "ns" << std::endl;
  // }

  // std::cout << "Total time: "
}

void CommsLatencyTestPublisherBq::shutdown() {
  std::cout << "Shutting down!" << std::endl;
}

}  // namespace jet

BALSA_QUEUE_MAIN_FUNCTION(jet::CommsLatencyTestPublisherBq)
