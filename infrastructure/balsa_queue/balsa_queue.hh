#pragma once

//%deps(yaml-cpp)

#include "infrastructure/comms/comms_factory.hh"
#include "infrastructure/config/config.hh"
#include "infrastructure/gonogo/gonogo.hh"
#include "infrastructure/time/timestamp.hh"

#include <yaml-cpp/yaml.h>

#include <stdint.h>
#include <memory>

namespace jet {

/// Balsa Queue: A base class that allows all processes on the balsa jet to share a common
/// structure and common event loop implementation.
class BalsaQ {
 public:
  uint loop_delay_microseconds = 10000;
  BalsaQ() = default;
  virtual void init(const Config& config) = 0;
  virtual void loop() = 0;
  virtual void shutdown() = 0;

  const std::string& name() const {
    return bq_name_;
  }

  GoNoGo& gonogo() {
    return gonogo_;
  }

  void set_name(const std::string& name) {
    bq_name_ = name;
  }

  void set_comms_factory(std::unique_ptr<CommsFactory> comms_factory);

 protected:
  std::unique_ptr<Publisher> make_publisher(const std::string& channel_name);
  std::unique_ptr<Subscriber> make_subscriber(const std::string& channel_name);
  Timestamp get_current_time();
  Timestamp last_msg_recvd_timestamp_;

 private:
  GoNoGo gonogo_ = GoNoGo();
  std::string bq_name_;
  std::unique_ptr<CommsFactory> comms_factory_;
};

}  // namespace jet
