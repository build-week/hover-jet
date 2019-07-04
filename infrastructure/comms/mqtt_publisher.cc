#include "mqtt_publisher.hh"

#include "infrastructure/time/time_utils.hh"

#include <crossguid/guid.hpp>

#include <cstdlib>

//%deps(paho-mqttpp3)
//%deps(crossguid)

namespace jet {

namespace {
constexpr int QOS = 0;
}

MqttPublisher::MqttPublisher(const std::string& channel_name) : Publisher(channel_name) {
  if (const char* env_addr = std::getenv("MQTT_ADDRESS")) {
    mqtt_server_address_ = env_addr;
  } else {
    throw std::runtime_error("Environment variable MQTT_ADDRESS must be set.");
  }

  mqtt_client_id_ = xg::newGuid().str();
  mqtt_client_ = std::make_unique<mqtt::async_client>(mqtt_server_address_, mqtt_client_id_);
  connect();
}

MqttPublisher::~MqttPublisher() {
  mqtt_client_->unsubscribe(channel_name_)->wait();
  mqtt_client_->stop_consuming();
  mqtt_client_->disconnect()->wait();
}

void MqttPublisher::connect() {
  try {
    mqtt::connect_options connection_options;
    mqtt_client_->connect(connection_options);
  } catch (const mqtt::exception& e) {
    print_error_rate_limited(++sequential_connect_failures_, "MQTT Publisher failed to connect. Will try again.");
  }
}

void MqttPublisher::reconnect() {
  try {
    mqtt_client_->reconnect();
  } catch (const mqtt::exception& e) {
    print_error_rate_limited(++sequential_connect_failures_, "MQTT Publisher failed to reconnect. Will try again.");
  }
}

void MqttPublisher::publish(Message& message) {
  ++sequence_number_;

  if (!mqtt_client_->is_connected()) {
    print_error_rate_limited(++sequential_failed_publishes_, "MQTT Publisher is not connected. Skipping publish.");
    reconnect();
    return;
  }

  message.header.sequence_number = sequence_number_;
  message.header.timestamp_ns = time::get_current_time();

  std::string data;


  auto serialize_start_time = time::get_current_time();
  message.serialize(data);
  uint64_t time_to_serialize = time::get_current_time() - serialize_start_time;
  if (time_to_serialize > 2000000){
    std::cout << "Time to serialize: " << std::fixed << std::setprecision(1) << time_to_serialize << "ns" << std::endl;
  }



  auto make_message_start_time = time::get_current_time();
  mqtt::message_ptr pubmsg = mqtt::make_message(channel_name_, data);
  uint64_t time_to_make_message = time::get_current_time() - make_message_start_time;
  if (time_to_make_message > 2000000){
    std::cout << "Time to make_message: " << std::fixed << std::setprecision(1) << time_to_make_message << "ns" << std::endl;
  }


  pubmsg->set_qos(QOS);


  auto mqtt_client_publish_start_time = time::get_current_time();
  mqtt_client_->publish(pubmsg)->wait();
  uint64_t time_to_mqtt_client_publish_time = time::get_current_time() - mqtt_client_publish_start_time;
  if (time_to_mqtt_client_publish_time > 2000000){
    std::cout << "Time to mqtt_client publish: " << std::fixed << std::setprecision(1) << time_to_mqtt_client_publish_time << "ns" << std::endl;
  }
}

void MqttPublisher::publish_raw(const std::string& data) {
  ++sequence_number_;

  if (!mqtt_client_->is_connected()) {
    print_error_rate_limited(++sequential_failed_publishes_, "MQTT Publisher is not connected. Skipping publish.");
    reconnect();
    return;
  }

  mqtt::message_ptr pubmsg = mqtt::make_message(channel_name_, data);
  pubmsg->set_qos(QOS);
  mqtt_client_->publish(pubmsg)->wait();
  sequential_failed_publishes_ = 0;
  sequential_connect_failures_ = 0;
}

}  // namespace jet
