
#include "mqtt_subscriber.hh"

#include <crossguid/guid.hpp>

#include <chrono>
#include <cstdlib>

//%deps(crossguid)
//%deps(paho-mqttpp3)
//%deps(message)

namespace jet {

MqttSubscriber::MqttSubscriber(const std::string& channel_name)
    : Subscriber(channel_name) {
  if (const char* env_addr = std::getenv("MQTT_ADDRESS")) {
    mqtt_server_address_ = env_addr;
  } else {
    std::cerr << "Environment variable MQTT_ADDRESS must be set." << std::endl;
  }

  mqtt_client_id_ = xg::newGuid().str();
  mqtt_client_ =
      std::make_unique<mqtt::async_client>(mqtt_server_address_, mqtt_client_id_);
  connect();
}

MqttSubscriber::~MqttSubscriber() {
  mqtt_client_->unsubscribe(channel_name_)->wait();
  mqtt_client_->stop_consuming();
  mqtt_client_->disconnect()->wait();
}

void MqttSubscriber::connect() {
  const int QOS = 1;
  try {
    mqtt::connect_options connection_options;
    connection_options.set_keep_alive_interval(20);
    connection_options.set_clean_session(true);
    mqtt_client_->connect(connection_options);
    mqtt_client_->start_consuming();
    mqtt_client_->subscribe(channel_name_, QOS);
  } catch (const mqtt::exception& exc) {
    std::cerr << exc.what() << std::endl;
  }
}

void MqttSubscriber::reconnect() {
  try {
    mqtt_client_->reconnect();
  } catch (const mqtt::exception& exc) {
    std::cerr << exc.what() << std::endl;
  }
}

bool MqttSubscriber::read(Message& message, const Duration& timeout) {
  if (!mqtt_client_->is_connected()) {
    std::cerr << "Not connected. Skipping read." << std::endl;
    reconnect();
    return false;
  }

  mqtt::const_message_ptr mqtt_message_ptr;
  bool error_code = mqtt_client_->try_consume_message_for(
      &mqtt_message_ptr, std::chrono::nanoseconds(timeout));
  if (!error_code) {
    return error_code;
  } else {
    message.deserialize(mqtt_message_ptr->get_payload_str());
  }
  return error_code;
}

bool MqttSubscriber::read_raw(std::string& message_data, const Duration& timeout) {
  if (!mqtt_client_->is_connected()) {
    std::cerr << "Not connected. Skipping read." << std::endl;
    reconnect();
    return false;
  }

  mqtt::const_message_ptr mqtt_message_ptr;
  bool error_code = mqtt_client_->try_consume_message_for(
      &mqtt_message_ptr, std::chrono::nanoseconds(timeout));
  if (!error_code) {
    return error_code;
  } else {
    message_data = mqtt_message_ptr->get_payload_str();
  }
  return error_code;
}

}  // namespace jet
