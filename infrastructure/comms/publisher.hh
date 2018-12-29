#pragma once

#include "schemas/message.hh"

#include <memory>
#include <string>

namespace jet {

class Publisher {
 public:
  Publisher(const std::string& channel_name) : channel_name_(channel_name) {
  }
  virtual void publish(Message& message) = 0;
  virtual void publish_raw(const std::string& data) = 0;

 protected:
  const std::string channel_name_;
  uint64_t sequence_number_{0};
};

using PublisherPtr = std::unique_ptr<Publisher>;

}  // namespace jet
