#pragma once

#include "infrastructure/balsa_queue/balsa_queue.hh"

namespace jet {

class TimesyncMasterBq : public BalsaQ {
 public:
  TimesyncMasterBq() = default;
  void init(const Config& config);
  void loop();
  void shutdown();

 private:
  PublisherPtr publisher_;
};

}  // namespace jet
