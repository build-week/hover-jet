#include "infrastructure/comms/mqtt_comms_factory.hh"
#include "mqtt_publisher.hh"
#include "mqtt_subscriber.hh"

//%deps(paho-mqttpp3)

namespace jet {

std::unique_ptr<Publisher> MqttCommsFactory::make_publisher(
    const std::string& channel_name) {
  return std::make_unique<MqttPublisher>(channel_name);
}

std::unique_ptr<Subscriber> MqttCommsFactory::make_subscriber(
    const std::string& channel_name) {
  return std::make_unique<MqttSubscriber>(channel_name);
}

}  // namespace jet
