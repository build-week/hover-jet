#pragma once

#include "infrastructure/balsa_queue/balsa_queue.hh"
#include "infrastructure/time/timesync_bq/client_timesync_message.hh"

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
  ClientTimesyncMessage client_message_;
};

}  // namespace jet
