#pragma once

#include "publisher.hh"
#include "subscriber.hh"

#include "mqtt/async_client.h"

#include <memory>
#include <string>

namespace jet {

class CommsFactory {
 public:
  CommsFactory() = default;
  virtual std::unique_ptr<Publisher> make_publisher(
      const std::string& channel_name) const = 0;
  virtual std::unique_ptr<Subscriber> make_subscriber(
      const std::string& channel_name) const = 0;
};

}  // namespace jet
