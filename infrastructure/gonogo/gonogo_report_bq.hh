#pragma once

#include "infrastructure/balsa_queue/balsa_queue.hh"
#include "infrastructure/gonogo/gonogo_message.hh"

#include <map>

namespace jet {

class GoNoGoReportBQ : public BalsaQ {
 public:
  GoNoGoReportBQ() = default;
  void init(const Config& config);
  void loop();
  void shutdown();

 private:
  SubscriberPtr gonogo_sub_;
  SubscriberPtr gonogo_state_sub_;

  std::map<std::string, bool> ready_from_bq_name_;
};

}  // namespace jet
