#include "mqtt_subscriber.hh"

#include <crossguid/guid.hpp>

#include <unistd.h>
#include <chrono>
#include <thread>

//%deps(crossguid)
//%deps(paho-mqttpp3)
//%deps(message)
//%deps(pthread)

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

  async_connect();
}

MqttSubscriber::~MqttSubscriber() {
  mqtt_client_->unsubscribe(channel_name_)->wait();
  mqtt_client_->stop_consuming();
  mqtt_client_->disconnect()->wait();
  if (connect_thread_ && connect_thread_->joinable()) {
    connect_thread_->join();
  }
}

void MqttSubscriber::async_connect() {
  if (connecting_) {
    return;
  }
  if (connect_thread_) {
    // @Ben: Leaving this here for a debug print in case the crash I saw happens again;
    // I believe it's linux trapping a thread fault if this join gets called after detach
    std::cout << "Attempting Join" << std::endl;
    connect_thread_->join();
  }
  connect_thread_ = std::make_unique<std::thread>([this]() { connect(); });
  connect_thread_->detach();
}

void MqttSubscriber::connect() {
  // Don't attempt to connect if we are currently connecting
  // Note: We believe that the mqtt client should be thread-safe,
  //       but it seems to deadlock if connect(...)->wait() called
  //       from two threads
  // TODO(jake, ben): Figure that out
  if (connecting_) {
    return;
  }
  connecting_ = true;
  const int QOS = 1;
  try {
    mqtt::connect_options connection_options;
    connection_options.set_keep_alive_interval(20);
    connection_options.set_clean_session(true);
    mqtt_client_->connect(connection_options)->wait();
    mqtt_client_->start_consuming();
    mqtt_client_->subscribe(channel_name_, QOS);
  } catch (const mqtt::exception& exc) {
    std::cerr << exc.what() << std::endl;
  }
  connecting_ = false;
}

bool MqttSubscriber::read(Message& message, const Duration& timeout) {
  if (!mqtt_client_->is_connected() || connecting_) {
    std::cerr << "Not connected. Skipping read." << std::endl;
    connect();
    return false;
  }

  mqtt::const_message_ptr mqtt_message_ptr;
  bool error_code = mqtt_client_->try_consume_message_for(
      &mqtt_message_ptr, std::chrono::nanoseconds(timeout));
  if (!error_code) {
    return error_code;
  } else if (!mqtt_message_ptr) {
    return false;
  } else {
    message.deserialize(mqtt_message_ptr->get_payload_str());
  }
  return error_code;
}

bool MqttSubscriber::read_raw(std::string& message_data, const Duration& timeout) {
  if (!mqtt_client_->is_connected()) {
    std::cerr << "Not connected. Skipping read." << std::endl;
    connect();
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
