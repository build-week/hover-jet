
#include "infrastructure/balsa_queue/balsa_queue_state_message.hh"
#include "infrastructure/comms/mqtt_comms_factory.hh"

#include <unistd.h>
#include <iomanip>
#include <memory>

float ns_to_ms(const jet::Duration& duration) {
  return duration / 10000.0;
}

int main(int argc, char* argv[]) {
  auto comms_factory = std::make_unique<jet::MqttCommsFactory>();
  auto subscriber = comms_factory->make_subscriber("bq_state");

  jet::BQStateMessage message;
  while (true) {
    if (subscriber->read(message, 100)) {
      std::cout << std::fixed << std::setprecision(1) << "[" << message.bq_instance_name
                << "] loop duration (ms) Min: " << ns_to_ms(message.loop_execution_times.min_time)
                << " Max: " << ns_to_ms(message.loop_execution_times.max_time)
                << " Median: " << ns_to_ms(message.loop_execution_times.median_time) << std::endl;
    }
  }
}
