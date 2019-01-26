#pragma once

#include <infrastructure/balsa_queue/balsa_queue.hh>

namespace jet {

class TimesyncClientBq : public BalsaQ {
 public:
  TimesyncClientBq() = default;
  void init(int argc, char *argv[]);
  void loop();
  void shutdown();

 private:
  PublisherPtr publisher_;
  SubscriberPtr subscriber_;
};

}  // namespace jet
