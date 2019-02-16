#pragma once

#include <infrastructure/balsa_queue/balsa_queue.hh>
#include <infrastructure/gonogo/gonogo_message.hh>

#include <map>

namespace jet {

class GoNoGoBQ : public BalsaQ {
 public:
  GoNoGoBQ() = default;
  void init(int argc, char *argv[]);
  void loop();
  void shutdown();

 private:
  SubscriberPtr gonogo_subscriber_;
  PublisherPtr gonogo_state_publisher_;

  std::map<std::string, GoNoGoMessage> go_no_go_states_;
};

}  // namespace jet
