#pragma once

#include "infrastructure/comms/publisher.hh"

#include <mqtt/async_client.h>

#include <memory>

namespace jet {

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

  void connect();
  void reconnect();
};

}  // namespace jet
