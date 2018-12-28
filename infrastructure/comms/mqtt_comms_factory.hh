#pragma once

#include "comms_factory.hh"

namespace jet {

class MqttCommsFactory : public CommsFactory {
 public:
  MqttCommsFactory() = default;
  std::unique_ptr<Publisher> make_publisher(const std::string& channel_name);
  std::unique_ptr<Subscriber> make_subscriber(const std::string& channel_name);
};

}  // namespace jet
