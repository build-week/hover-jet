#pragma once

#include <infrastructure/balsa_queue/balsa_queue.hh>

namespace jet {

class TimesyncMasterBq : public BalsaQ {
 public:
  TimesyncMasterBq() = default;
  void init(int argc, char *argv[]);
  void loop();
  void shutdown();

 private:
  PublisherPtr publisher_;
};

}  // namespace jet
