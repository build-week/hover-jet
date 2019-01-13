#pragma once

#include "infrastructure/comms/comms_factory.hh"
#include "infrastructure/time/timestamp.hh"

#include <stdint.h>
#include <memory>

namespace jet {

/// Balsa Queue: A base class that allows all processes on the balsa jet to share a common
/// structure and common event loop implementation.
class BalsaQ {
 public:
  uint loop_delay_microseconds = 10000;
  BalsaQ() = default;
  virtual void init(int argc, char *argv[]) = 0;
  virtual void loop() = 0;
  virtual void shutdown() = 0;

  void set_comms_factory(std::unique_ptr<CommsFactory> comms_factory);

 protected:
  std::unique_ptr<Publisher> make_publisher(const std::string& channel_name);
  std::unique_ptr<Subscriber> make_subscriber(const std::string& channel_name);
  Timestamp get_current_time();

 private:
  std::unique_ptr<CommsFactory> comms_factory_;
};

}  // namespace jet
