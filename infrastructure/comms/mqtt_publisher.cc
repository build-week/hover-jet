#include "mqtt_publisher.hh"

#include <crossguid/guid.hpp>

#include <chrono>
#include <cstdlib>

<<<<<<< HEAD
=======
#include <iostream>

>>>>>>> Removing extra includes
//%deps(paho-mqttpp3)
//%deps(crossguid)

namespace jet {

namespace {
constexpr int QOS = 1;
}

MqttPublisher::MqttPublisher(const std::string& channel_name) : Publisher(channel_name) {
  if (const char* env_addr = std::getenv("MQTT_ADDRESS")) {
    mqtt_server_address_ = env_addr;
  } else {
    throw std::runtime_error("Environment variable MQTT_ADDRESS must be set.");
  }

  mqtt_client_id_ = xg::newGuid().str();
  mqtt_client_ =
      std::make_unique<mqtt::async_client>(mqtt_server_address_, mqtt_client_id_);
  connect();
}

MqttPublisher::~MqttPublisher() {
  mqtt_client_->unsubscribe(channel_name_)->wait();
  mqtt_client_->stop_consuming();
  mqtt_client_->disconnect()->wait();
}

void MqttPublisher::connect() {
  std::cout << "Connecting." << std::endl;
  try {
    mqtt::connect_options connection_options;
    mqtt_client_->connect(connection_options);
  } catch (const mqtt::exception& e) {
    std::cerr << e.what() << std::endl;
  }
}

void MqttPublisher::reconnect() {
  std::cout << "Reconnecting." << std::endl;
  try {
    mqtt_client_->reconnect();
  } catch (const mqtt::exception& e) {
    std::cerr << e.what() << std::endl;
  }
}

void MqttPublisher::publish(Message& message) {
  ++sequence_number_;

  if (!mqtt_client_->is_connected()) {
    std::cerr << "Not connected. Skipping publish." << std::endl;
    reconnect();
    return;
  }

  message.header.sequence_number = sequence_number_;

  std::string data;
  message.serialize(data);
  mqtt::message_ptr pubmsg = mqtt::make_message(channel_name_, data);
  pubmsg->set_qos(QOS);
  mqtt_client_->publish(pubmsg)->wait();
}

void MqttPublisher::publish_raw(const std::string& data) {
  ++sequence_number_;

  if (!mqtt_client_->is_connected()) {
    std::cerr << "Not connected. Skipping publish." << std::endl;
    reconnect();
    return;
  }

  mqtt::message_ptr pubmsg = mqtt::make_message(channel_name_, data);
  pubmsg->set_qos(QOS);
  mqtt_client_->publish(pubmsg)->wait();
}

}  // namespace jet
