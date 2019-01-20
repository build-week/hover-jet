
#pragma once

#include "infrastructure/balsa_queue/balsa_queue.hh"
#include "infrastructure/comms/mqtt_comms_factory.hh"
#include "infrastructure/comms/schemas/message.hh"

#include "helpers/rate_helper.hh"

#include <memory>
#include <unistd.h>

namespace jet {

class MessageHelperBQ : public BalsaQ {
 public:
  MessageHelperBQ();
  void init(int argc, char *argv[]);
  void loop();
  void shutdown();

 private:
  std::string channel_;
  SubscriberPtr subscriber_;
  std::unique_ptr<utils::RateHelper> rate_helper_;
};

}  // namespace jet
