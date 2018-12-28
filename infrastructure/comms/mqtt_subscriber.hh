#pragma once

#include "infrastructure/comms/subscriber.hh"
#include "infrastructure/time/duration.hh"

#include <mqtt/async_client.h>

#include <memory>

namespace jet {

class MqttSubscriber : public Subscriber {
 public:
  MqttSubscriber(const std::string& channel_name);
  ~MqttSubscriber();
  bool read(Message& message, const Duration& timeout);
  bool read_raw(std::string& message_data, const Duration& timeout);

 private:
  std::string mqtt_server_address_;
  std::string mqtt_client_id_;
  std::unique_ptr<mqtt::async_client> mqtt_client_;

  void connect();
  void reconnect();
};

}  // namespace jet
