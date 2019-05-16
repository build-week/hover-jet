#pragma once

#include "infrastructure/comms/publisher.hh"

#include <mqtt/async_client.h>

#include <memory>

namespace jet {

namespace {
inline constexpr int ERROR_MESSAGE_PERIOD = 100;
}

inline void print_error_rate_limited(uint32_t error_count, std::string_view message) {
  if (error_count == 1 || error_count % ERROR_MESSAGE_PERIOD == 0) {
    std::cerr << message << std::endl;
  }
}

class MqttPublisher : public Publisher {
 public:
  MqttPublisher(const std::string& channel_name);
  ~MqttPublisher();
  void publish(Message& message);
  void publish_raw(const std::string& data);

 private:
  std::string mqtt_server_address_;
  std::string mqtt_client_id_;
  std::unique_ptr<mqtt::async_client> mqtt_client_;

  uint32_t sequential_connect_failures_{0};
  uint32_t sequential_failed_publishes_{0};

  void connect();
  void reconnect();
};

}  // namespace jet
