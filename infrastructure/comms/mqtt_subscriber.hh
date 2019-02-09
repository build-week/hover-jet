#pragma once

#include "infrastructure/comms/subscriber.hh"
#include "infrastructure/time/duration.hh"

#include <mqtt/async_client.h>

#include <atomic>
#include <memory>

namespace jet {

class MqttSubscriber : public Subscriber {
 public:
  MqttSubscriber(const std::string& channel_name);
  ~MqttSubscriber();
  bool read(Message& message, const Duration& timeout);
  bool read_raw(std::string& message_data, const Duration& timeout);
  bool read_latest(Message& message, const Duration& timeout);

 private:
  std::string mqtt_server_address_;
  std::string mqtt_client_id_;
  std::unique_ptr<mqtt::async_client> mqtt_client_;
  std::unique_ptr<std::thread> connect_thread_;
  std::atomic<bool> connecting_;

  void async_connect();
  void connect();
  bool get_mqtt_message(mqtt::const_message_ptr& mqtt_message_ptr, const Duration& timeout);
};

}  // namespace jet
