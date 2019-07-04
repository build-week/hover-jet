#pragma once

#include "infrastructure/time/duration.hh"
#include "infrastructure/comms/schemas/message.hh"

#include <memory>
#include <string>

namespace jet {

class Subscriber {
 public:
  Subscriber(const std::string& channel_name) : channel_name_(channel_name){};
  virtual bool read(Message& message, const Duration& timeout) = 0;
  virtual bool read_raw(std::string& message_data, const Duration& timeout) = 0;
  virtual bool read_latest(Message& message, const Duration& timeout) = 0;

 protected:
  const std::string channel_name_;
};

using SubscriberPtr = std::unique_ptr<Subscriber>;

}  // namespace jet
